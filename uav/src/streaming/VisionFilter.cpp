#include "VisionFilter.h"
#include "compile_info.h"

static void constructor() __attribute__((constructor));

void constructor() {
  compile_info("FlairVisionFilter");
}

void saveToJpeg(flair::core::Image* src_img,std::string filename,PictureFormat_t input_format,PictureFormat_t output_format,unsigned char compression_level) {
	printf("saveToJpeg todo\n");
    //if(!cvSaveImage(filename.c_str(),src_img)) printf("Could not save.\n");
}

size_t ConvertToJpeg(flair::core::Image* src_img,char* output_buffer,size_t output_buffer_size,PictureFormat_t input_format,PictureFormat_t output_format,unsigned char compression_level) {
    printf("ConvertToJpeg todo\n");

    return 0;
}

bool InitVisionFilter(std::string args) {
  printf("Using default vision lib filter, all functions are not ready!\n");
	return true;
}

void CloseVisionFilter(void) {
  
}

char* AllocFunction(ssize_t size) {
    return (char*)malloc(size);
}

void FreeFunction(char* buffer) {
    free(buffer);
}
