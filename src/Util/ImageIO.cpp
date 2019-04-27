/*!
  @file
  @author Shin'ichiro Nakaoka
  @author Hisashi Ikari
*/

#include "ImageIO.h"
#include "Exception.h"
#include <fmt/format.h>
#include <boost/algorithm/string/predicate.hpp>
#include <png.h>

extern "C" {
#define XMD_H
#include <jpeglib.h>
}

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

void throwLoadException(const string& filename, const std::string& description)
{
    exception_base exception;
    exception << error_info_message(
        fmt::format("Image file \"{0}\" cannot be loaded. {1}", filename, description));
    BOOST_THROW_EXCEPTION(exception);
}


void throwSaveException(const string& filename, const std::string& description)
{
    exception_base exception;
    exception << error_info_message(
        fmt::format("Image cannot be save to \"{0}\". {1}", filename, description));
    BOOST_THROW_EXCEPTION(exception);
}

    
void loadPNG(Image& image, const std::string& filename, bool isUpsideDown)
{
    FILE* fp = 0;
    fp = fopen(filename.c_str(), "rb");
    if(!fp){
        throwLoadException(filename, strerror(errno));
    }
    png_size_t number = 8;
    png_byte header[8];
    int is_png;
        
    size_t n = fread(header, 1, number, fp);
    if(n != number){
        if(fp){
            fclose(fp);
        }
        throwLoadException(filename, "The file is not the PNG format.");
    }
    is_png = !png_sig_cmp(header, 0, number);
    if(!is_png){
        if(fp){
            fclose(fp);
        }
        throwLoadException(filename, "The file is not the PNG format.");
    }
        
    png_structp pPng;
    pPng = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if(!pPng){
        if(fp){
            fclose(fp);
        }
        throwLoadException(filename, "Failed to create png_struct.");
    }
        
    png_infop pInfo;            
    pInfo = png_create_info_struct(pPng);
    if(!pInfo){
        png_destroy_read_struct( &pPng, NULL, NULL );
        if(fp){
            fclose(fp);
        }
        throwLoadException(filename, "Failed to create png_info");
    }
        
    png_init_io(pPng, fp);
    png_set_sig_bytes(pPng, (int)number);
        
    png_read_info(pPng, pInfo);
    png_uint_32 width   = png_get_image_width (pPng, pInfo);
    png_uint_32 height  = png_get_image_height(pPng, pInfo);
    png_byte color_type = png_get_color_type  (pPng, pInfo);
    png_byte depth = png_get_bit_depth        (pPng, pInfo);
        
    if(png_get_valid( pPng, pInfo, PNG_INFO_tRNS)){
        png_set_tRNS_to_alpha(pPng);
    }
    if(depth < 8){
        png_set_packing(pPng);
    }
        
    switch (color_type) {
    case PNG_COLOR_TYPE_GRAY:
        image.setSize(width, height, 1);
        if(depth < 8){
            png_set_expand_gray_1_2_4_to_8(pPng);
        }
        break;
            
    case PNG_COLOR_TYPE_GRAY_ALPHA:
        image.setSize(width, height, 2);
        if(depth == 16){
            png_set_strip_16(pPng);
        }
        break;
            
    case PNG_COLOR_TYPE_RGB:
        image.setSize(width, height, 3);
        if(depth == 16){
            png_set_strip_16(pPng);
        }
        break;
            
    case PNG_COLOR_TYPE_RGB_ALPHA:
        image.setSize(width, height, 4);
        if(depth == 16){
            png_set_strip_16(pPng);
        }
        break;
            
    case PNG_COLOR_TYPE_PALETTE:
        png_set_palette_to_rgb(pPng);
        image.setSize(width, height, 3);
        break;
            
    default:
        image.reset();
        throwLoadException(filename, "Unsupported color type.");
    }
        
    png_read_update_info(pPng, pInfo);        
        
    unsigned char** row_pointers;
    row_pointers = (png_bytepp)malloc(height * sizeof(png_bytep)); 
    png_uint_32 rowbytes = png_get_rowbytes(pPng, pInfo);
        
    unsigned char* pixels = image.pixels();
    if(isUpsideDown){
        for(png_uint_32 i = 0; i < height; ++i) {
            row_pointers[i] = &(pixels[(height - i - 1) * rowbytes]);
        }
    } else {
        for(png_uint_32 i = 0; i < height; ++i) {
            row_pointers[i] = &(pixels[i * rowbytes]);
        }
    }
    png_read_image(pPng, row_pointers);  
        
    free(row_pointers);
    png_destroy_read_struct(&pPng, &pInfo, NULL);
        
    fclose(fp);
}


void savePNG(const Image& image, const std::string& filename, bool isUpsideDown)
{
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if(!png_ptr){
        throwSaveException(filename, "Internal error.");
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if(!info_ptr){
       png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
       throwSaveException(filename, "Internal error.");
    }

    FILE* fp = fopen(filename.c_str(), "wb");
    if(!fp){
        png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        throwSaveException(filename, strerror(errno));
    }
    
    if(setjmp(png_jmpbuf(png_ptr))){
        png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        fclose(fp);
        throwSaveException(filename, "Internal error.");
    }

    png_init_io(png_ptr, fp);

    int color_type;
    if(image.numComponents() >= 3){
        if(image.hasAlphaComponent()){
            color_type = PNG_COLOR_TYPE_RGB_ALPHA;
        } else {
            color_type = PNG_COLOR_TYPE_RGB;
        }
    } else {
        if(image.hasAlphaComponent()){
            color_type = PNG_COLOR_TYPE_GRAY_ALPHA;
        } else {
            color_type = PNG_COLOR_TYPE_GRAY;
        }
    }

    const int height = image.height();
    const int width = image.width();
     
    png_set_IHDR(png_ptr, info_ptr,
                 width, height, 8, color_type,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png_ptr, info_ptr);

#if defined(_MSC_VER) && _MSC_VER < 2000
    png_bytep* row_pointers;
    row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
#else
    png_bytep row_pointers[height];
#endif

    for(int i=0; i < height; ++i){
        row_pointers[i] = const_cast<unsigned char*>(image.pixels()) + width * image.numComponents() * i;
    }

    png_write_image(png_ptr, row_pointers);

    png_write_end(png_ptr, info_ptr);
    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);

    fclose(fp);

#if defined(_MSC_VER) && _MSC_VER < 2000
    free(row_pointers);
#endif
}

    
void loadJPEG(Image& image, const std::string& filename, bool isUpsideDown)
{
    FILE* fp = 0;
    fp = fopen(filename.c_str(), "rb");
    if(!fp){
        throwLoadException(filename, strerror(errno));
    }
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
        
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, fp);
        
    (void)jpeg_read_header(&cinfo, TRUE);
    (void)jpeg_start_decompress(&cinfo);
    image.setSize(cinfo.output_width, cinfo.output_height, cinfo.output_components);
        
    unsigned char* pixels = image.pixels();
    const int h = image.height();
    const int w = image.width();
        
    JSAMPARRAY row_pointers = (JSAMPARRAY)malloc(sizeof(JSAMPROW) * image.height());
    if(isUpsideDown){
        for(int i = 0; i < h; ++i) { 
            row_pointers[i] = &(pixels[(h - i - 1) * cinfo.output_components * w]);
        }
    } else {
        for(int i = 0; i < h; ++i) { 
            row_pointers[i] = &(pixels[i * cinfo.output_components * w]);
        }
    }
    while(cinfo.output_scanline < cinfo.output_height){
        jpeg_read_scanlines(&cinfo, row_pointers + cinfo.output_scanline, cinfo.output_height - cinfo.output_scanline);
    }
        
    free(row_pointers);
    (void)jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
        
    fclose(fp);
}


void loadTGA(Image& image, const std::string& filename, bool isUpsideDown)
{
    FILE* fp = 0;
    fp = fopen(filename.c_str(), "rb");
    if(!fp){
        throwLoadException(filename, strerror(errno));
    }

    unsigned char header[12]={0,0,2,0,0,0,0,0,0,0,0,0};
    unsigned char header_buf[12];
    unsigned char header_buf2[6];

    if( fread(header_buf,1,sizeof(header),fp)!=sizeof(header) ||
        memcmp(header,header_buf,sizeof(header))!=0 ||
        fread(header_buf2, 1, sizeof(header_buf2), fp)!=sizeof(header_buf2) ){
        fclose(fp);
        throwLoadException(filename, "The file is not the Uncompressed TGA format.");
    }

    unsigned int width = header_buf2[1] * 256 + header_buf2[0];
    unsigned int height = header_buf2[3] * 256 + header_buf2[2];
    unsigned int bitPerPixel = header_buf2[4];
    unsigned int bytesPerPixel = bitPerPixel/8;

    if( width<=0 || height<=0 || (bytesPerPixel!=3 && bytesPerPixel!=4) )
    {
        fclose(fp);
        image.setSize(0, 0);
        return;
    }

    image.setSize(width, height, bytesPerPixel);

    unsigned char* pixels = image.pixels();
    for(unsigned int i=0; i<height; i++){
        unsigned int row;
        if(isUpsideDown){
            row = (height-1-i) * width * bytesPerPixel;
        }else{
            row = i * width * bytesPerPixel;
        }

        for(unsigned int j=0, k=row; j<width; j++){

            unsigned char imageBuf[4];
            if( fread(imageBuf, 1, bytesPerPixel, fp)!=bytesPerPixel ){
                fclose(fp);
                throwLoadException(filename, "Internal error." );
            }

            pixels[k++] = imageBuf[2];
            pixels[k++] = imageBuf[1];
            pixels[k++] = imageBuf[0];
            if(bytesPerPixel==4)
                pixels[k++] = imageBuf[3];
        }
    }

    fclose (fp);

}

}


ImageIO::ImageIO()
{
    isUpsideDown_ = false;
}


void ImageIO::load(Image& image, const std::string& filename)
{
    if(iends_with(filename, "png")){
        loadPNG(image, filename, isUpsideDown_);
    } else if(iends_with(filename, "jpg") || iends_with(filename, "jpeg")) {
        loadJPEG(image, filename, isUpsideDown_);
    } else if(iends_with(filename, "tga")){
        loadTGA(image, filename, isUpsideDown_);
    } else {
        throwLoadException(filename, "The image format type is not supported.");
    }
}


void ImageIO::save(const Image& image, const std::string& filename)
{
    if(iends_with(filename, "png")){
        savePNG(image, filename, isUpsideDown_);
    } else {
        throwSaveException(filename, "unsupported image format.");
    }
}
