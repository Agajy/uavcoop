#include <string>

namespace flair {
    namespace core {
        class Image;
    }
}


/*!
* \brief InitVisionFilter
*
* \return true
*/

bool InitVisionFilter(std::string args);

void CloseVisionFilter(void);

enum class PictureFormat_t {
                Gray,/*!< gray 8 bits */
                RGB,/*!< rgb 24 bits */
                YUV_422ile,/*!< YUV 4:2:2 interleaved */
                YUV_422p,/*!< YUV 4:2:2 planer */
                } ;
								
void saveToJpeg(flair::core::Image* src_img,std::string filename,PictureFormat_t input_format,PictureFormat_t output_format,unsigned char compression_level=95);

size_t ConvertToJpeg(flair::core::Image* src_img,char* output_buffer,size_t output_buffer_size,PictureFormat_t input_format,PictureFormat_t output_format,unsigned char compression_level=95);

//for amrv7a, these functions use CMEM
char* AllocFunction(ssize_t size);
void FreeFunction(char* buffer);
