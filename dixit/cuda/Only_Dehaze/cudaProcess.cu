#include <cuda.h>
#include <cuda_runtime.h>
#include <iostream>
#include <ctime>
#include <chrono>

float measure_kernel_execution_time(cudaEvent_t start, cudaEvent_t stop)
{
    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);
    return milliseconds;
}

__global__ void haze_removal_kernel(const unsigned char *src, int rows, int cols, int channels, int A, const unsigned char *tx, unsigned char *result_img)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;

    if (idx < cols && idy < rows)
    {
        int index = (idy * cols + idx) * channels;
        float max_t = static_cast<float>(tx[idy * cols + idx]) / 255.0f;
        if (max_t < 0.5f)
        {
            max_t = 0.5f;
        }

        for (int c = 0; c < channels; ++c)
        {
            int value_num = src[index + c];
            result_img[index + c] = static_cast<unsigned char>((value_num - A) / max_t + A);
        }


    }
}

__global__ void calculate_tx_kernel(int rows, int cols, int channels, float A, unsigned char *dark_channel_data, unsigned char *tx_data)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;

    if (idx < cols && idy < rows)
    {

        float dark_channel_val = static_cast<float>(dark_channel_data[idy * cols + idx]) / A;

        tx_data[idy * cols + idx] = static_cast<unsigned char>(255.0f * (1.0f - (0.95f * dark_channel_val)));
    }
}

__global__ void sortIndicesByDarkChannel(const unsigned char *darkChannel, int *sortedIndices, int numRows, int numCols)
{
    int row = blockIdx.y * blockDim.y + threadIdx.y;
    int col = blockIdx.x * blockDim.x + threadIdx.x;
    int idx = row * numCols + col;

    if (row < numRows && col < numCols)
    {
        sortedIndices[idx] = idx;
    }
    __syncthreads();

    if (row < numRows && col < numCols)
    {
        int temp = sortedIndices[idx];
        int j = idx - 1;
        while (j >= 0 && darkChannel[temp] > darkChannel[sortedIndices[j]])
        {
            sortedIndices[j + 1] = sortedIndices[j];
            j--;
        }
        sortedIndices[j + 1] = temp;
    }
}

__global__ void findMaxValue(const unsigned char *imageData, const int *sortedIndices, int numRows, int numCols, float *maxVal)
{
    // Calculate the number of elements for 1%
    int numElements = numRows * numCols;
    int onePercent = numElements;

    // Each thread handles one element
    int idx = threadIdx.x + blockIdx.x * blockDim.x;

    // Initialize shared memory to store intermediate maximum values
    __shared__ float s_maxVal[256];
    s_maxVal[threadIdx.x] = 0.0f;

    // Fetch the indices from sortedIndices and find their corresponding pixels in the RGB image
    if (idx < onePercent)
    {
        int sortedIndex = sortedIndices[idx];
        unsigned char r = imageData[sortedIndex * 3];
        unsigned char g = imageData[sortedIndex * 3 + 1];
        unsigned char b = imageData[sortedIndex * 3 + 2];

        // Calculate the maximum value for this thread
        float threadMax = fmaxf(fmaxf(r, g), b);

        // Store the maximum value in shared memory
        s_maxVal[threadIdx.x] = threadMax;
    }

    // Synchronize threads to ensure all values are stored in shared memory
    __syncthreads();

    // Perform parallel reduction to find the maximum value across all threads
    #pragma unroll
    for (int stride = blockDim.x / 2; stride > 0; stride >>= 1)
    {
        if (threadIdx.x < stride)
        {
            s_maxVal[threadIdx.x] = fmaxf(s_maxVal[threadIdx.x], s_maxVal[threadIdx.x + stride]);
        }
        __syncthreads();
    }

    // Store the final maximum value to global memory
    if (threadIdx.x == 0)
    {
        atomicMax(reinterpret_cast<unsigned int *>(maxVal), *reinterpret_cast<unsigned int *>(&s_maxVal[0]));
    }
}

__global__ void min_channel_kernel(unsigned char *src_data, unsigned char *min_mat_data, int rows, int cols, int channels)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    int index = idy * cols + idx;

    if (idx < cols && idy < rows)
    {
        int min_val = 255;
        #pragma unroll
        for (int c = 0; c < channels; ++c)
        {
            int val = src_data[index * channels + c];
            min_val = min(min_val, val);
        }
        min_mat_data[index] = min_val;
    }
}

__global__ void min_filter_kernel(unsigned char *min_mat_expansion_data, unsigned char *dark_channel_mat_data, int rows, int cols, int border)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    int m = idy + border;
    int n = idx + border;

    if (m < rows - border && n < cols - border)
    {
        int min_num = 255;
        #pragma unroll
        for (int i = -border; i <= border; ++i)
        {
            #pragma unroll
            for (int j = -border; j <= border; ++j)
            {
                int val_roi = min_mat_expansion_data[(m + i) * cols + n + j];
                min_num = min(min_num, val_roi);
            }
        }
        dark_channel_mat_data[(m - border) * (cols - 2 * border) + (n - border)] = min_num;
    }
}

__global__ void padImage(unsigned char *input, unsigned char *output, int rows, int cols, int borderSize)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;

    // Padding top border
    if (i < cols && j < borderSize)
    {
        int idx_out = i + borderSize + (j * (cols + 2 * borderSize) + borderSize);
        int idx_in = i + j * cols;
        output[idx_out] = input[idx_in];
    }

    // Padding bottom border
    if (i < cols && j >= rows - borderSize)
    {
        int idx_out = i + borderSize + ((j + borderSize * 2) * (cols + 2 * borderSize) + borderSize);
        int idx_in = i + (rows - borderSize - 1) * cols + (j - (rows - borderSize)) * cols;
        output[idx_out] = input[idx_in];
    }

    // Padding left border
    if (i < borderSize && j < rows)
    {
        int idx_out = i + (j * (cols + 2 * borderSize));
        int idx_in = j * cols;
        output[idx_out] = input[idx_in];
    }

    // Padding right border
    if (i >= cols && i < cols + borderSize && j < rows)
    {
        int idx_out = (i + borderSize) + (j * (cols + 2 * borderSize)) + borderSize;
        int idx_in = (i - cols + 1) + j * cols;
        output[idx_out] = input[idx_in];
    }

    if (i < cols && j < rows)
    {
        int idx_in = i + j * cols;
        int idx_out = (i + borderSize) + (j + borderSize) * (cols + 2 * borderSize);
        output[idx_out] = input[idx_in];
    }
}



void print_data(unsigned char *data, int rows, int cols, int channels)
{
    std::cout << "===============================start=================================" << std::endl;
    for (int i = 0; i < rows - 400; ++i)
    {
        for (int j = 0; j < cols - 400; ++j)
        {
            for (int c = 0; c < channels; ++c)
            {
                std::cout << static_cast<int>(data[(i * cols + j) * channels + c]) << " ";
            }
            std::cout << "| ";
        }
        std::cout << std::endl;
    }

    std::cout << "===============================end==================================" << std::endl;
}








unsigned char* Dehaze_CUDA(unsigned char *src_ptr, int rows, int cols, int channels, int border)
{


    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    // Record the start event
    cudaEventRecord(start);
    //Varible Decleration
    unsigned char *src_data;
    unsigned char *min_mat_data;
    unsigned char *dark_channel_mat_data;
    unsigned char *min_mat_expansion_data;
    int* sortedIndices;
    float* A;
    unsigned char* dev_tx_data = 0;
    unsigned char* result;
    unsigned char * result_img;


    //cudaMallocs
    cudaMalloc(&src_data, rows * cols * channels * sizeof(unsigned char));
    cudaMalloc(&min_mat_data, rows * cols * sizeof(unsigned char));
    cudaMalloc(&dark_channel_mat_data, rows * cols * sizeof(unsigned char));


    //cudaMemcpys
    cudaMemcpy(src_data, src_ptr, rows * cols * channels * sizeof(unsigned char), cudaMemcpyHostToDevice);



    //Thread and grids decleartion 
    dim3 threadsPerBlock(32, 32);
    dim3 numBlocks((int)ceil((cols + threadsPerBlock.x - 1) / threadsPerBlock.x), (int)ceil((rows + threadsPerBlock.y - 1) / threadsPerBlock.y));



    // kernels of dark channel
    min_channel_kernel<<<numBlocks, threadsPerBlock>>>(src_data, min_mat_data, rows, cols, channels);
    cudaDeviceSynchronize();


    cudaMalloc(&min_mat_expansion_data, (rows + 2 * border) * (cols + 2 * border) * sizeof(unsigned char));

    padImage<<<numBlocks, threadsPerBlock>>>(min_mat_data, min_mat_expansion_data, rows, cols, border);
    cudaDeviceSynchronize();

    min_filter_kernel<<<numBlocks, threadsPerBlock>>>(min_mat_expansion_data, dark_channel_mat_data, rows + 2 * border, cols + 2 * border, border);
    cudaDeviceSynchronize();



    cudaFree(min_mat_data);
    cudaFree(min_mat_expansion_data);


    // kernels of A estimation
    cudaMalloc(&sortedIndices, rows * cols * sizeof(int));
    sortIndicesByDarkChannel<<<numBlocks, threadsPerBlock>>>(dark_channel_mat_data, sortedIndices, rows, cols);
    cudaDeviceSynchronize();

    

    cudaMallocManaged(&A, sizeof(float));
    
    *A = 0.00f;
    findMaxValue<<<numBlocks, threadsPerBlock>>>(src_data, sortedIndices, rows, cols, A);
    cudaDeviceSynchronize();

    cudaFree(sortedIndices);



    // kernels of tx map
    cudaMalloc((void**)&dev_tx_data, rows * cols * channels * sizeof(unsigned char));

    calculate_tx_kernel<<<numBlocks, threadsPerBlock>>>( rows, cols, channels, *A, dark_channel_mat_data, dev_tx_data);
    cudaDeviceSynchronize();




    // Kernels for Dehaze formula
    cudaMalloc((void**)&result, rows * cols * channels * sizeof(unsigned char));

    
    haze_removal_kernel<<<numBlocks, threadsPerBlock>>>(src_data, rows, cols, channels, *A, dev_tx_data, result);
    
    cudaHostAlloc((void **)&result_img, rows * cols * channels * sizeof(unsigned char), cudaHostAllocMapped);
    
    cudaMemcpy(result_img, result, rows * cols * channels * sizeof(unsigned char), cudaMemcpyDeviceToHost);


    cudaFree(src_data);
    cudaFree(dark_channel_mat_data);
    cudaFree(A);
    cudaFree(dev_tx_data);
    cudaFree(result);


    float milliseconds = measure_kernel_execution_time(start, stop);
    std::cout << "Kernel execution time: " << milliseconds << " ms" << std::endl;

    // Release CUDA events
    cudaEventDestroy(start);
    cudaEventDestroy(stop);


    return result_img;
}



int main(int argc, char *argv[])
{
	FILE *fp_in, *fp_out;
	
	unsigned char * ori;
	
	cudaHostAlloc((void **)&ori, sizeof(unsigned char)*450*600*3, cudaHostAllocMapped);
	

	auto fstart = std::chrono::steady_clock::now();
	fp_in = fopen("sample.bmp", "rb");
	fread(ori, 450*600*3, 1, fp_in);
	fclose(fp_in);
	

	
	unsigned char *dehaze = Dehaze_CUDA(ori, 450, 600, 3, 3);
	



	fp_out = fopen("output.bmp", "wb");
	fwrite(dehaze, 450*600*3, 1, fp_out);
	fclose(fp_out);
	
	auto fend = std::chrono::steady_clock::now();

        // Calculate the duration of the function execution
        auto fduration = std::chrono::duration_cast<std::chrono::milliseconds>(fend - fstart);

        // Print the duration
        std::cout << "Function execution duration: " << fduration.count() << " milliseconds" << std::endl;
}
