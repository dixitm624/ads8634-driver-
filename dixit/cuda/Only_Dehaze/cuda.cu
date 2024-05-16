#include<stdio.h>
#include<cuda.h>
#include<cuda_runtime.h>

#include<iostream>
#include<ctime>
#include<chrono>

unsigned char* Dehaze_CUDA(unsigned char *src_ptr,int rows, int cols, int channnels, int border)
{
    cudaEvent_t
}

int main(int argc, char *argv[]){
    FILE *fp_in , *fp_out;

    unsigned char * ori;

    cudaHostAlloc((void **)&ori, sizeof(unsigned char)*40*600*3, cudaHostAllocMapped);
    
    auto fstart = std::chrono::steady_clock::now();
    fp_in = fopen("Sample.bmp", "rb");
    fread(ori, 450*600*3, 1, fp_in);
    fclose(fp_in);

    unsigned char *dehaze = Dehaze_CUDA(ori, 450, 600, 3, 3);

    fp_out = fopen("output.bmp", "wb");
    fwrite(dehaze, 450*600*3, 1, fp_out);
    fclose(fp_out);

    auto fend = std::chrono::steady_clock::now();

        auto fduration = std::chrono::duration_cast<std::chrono::milliseconds>(fend -  fstart);

        std::cout << "Function Excetion duration: " << fduration.count() << " milliseconds" << std::endl;

}

