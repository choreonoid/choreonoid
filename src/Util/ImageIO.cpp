#include "ImageIO.h"
#include "UTF8.h"
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include <png.h>
#include <algorithm>

extern "C" {
#define XMD_H
#include <jpeglib.h>
}

#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = stdx::filesystem;

namespace {

bool loadPNG(Image& image, const std::string& filename, bool isUpsideDown, ostream& os)
{
    FILE* fp = fopen(fromUTF8(filename).c_str(), "rb");

    if(!fp){
        os << format(_("Image file \"{0}\" cannot be loaded. {1}"), filename, strerror(errno)) << endl;
        return false;
    }

    png_size_t number = 8;
    png_byte header[8];
    int is_png;
        
    size_t n = fread(header, 1, number, fp);
    if(n != number || png_sig_cmp(header, 0, number) != 0){
        fclose(fp);
        os << format(_("Image file \"{0}\" is not the PNG format."), filename) << endl;
        return false;
    }

    png_structp pPng = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if(!pPng){
        fclose(fp);
        os << format(_("Failed to create png_struct in loading \"{0}\""), filename) << endl;
        return false;
    }
        
    png_infop pInfo = png_create_info_struct(pPng);
    if(!pInfo){
        png_destroy_read_struct(&pPng, NULL, NULL);
        fclose(fp);
        os << format(_("Failed to create png_info in loading \"{0}\""), filename) << endl;
        return false;
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
        os << format(_("Image file \"{0}\" cannot be loaded because its color type is not supported."), filename)
           << endl;
    }
        
    png_read_update_info(pPng, pInfo);        
        
    vector<unsigned char*> row_pointers(height * sizeof(png_bytep));
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
    png_read_image(pPng, &row_pointers[0]);
    
    png_destroy_read_struct(&pPng, &pInfo, NULL);
    fclose(fp);

    return true;
}


bool savePNG(const Image& image, const std::string& filename, bool isUpsideDown, ostream& os)
{
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if(!png_ptr){
        os << format(_("Internal error in saving \"{0}\"."), filename) << endl;
        return false;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if(!info_ptr){
        png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        os << format(_("Internal error in saving \"{0}\"."), filename) << endl;
        return false;
    }

    FILE* fp = fopen(filename.c_str(), "wb");
    if(!fp){
        png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        os << format(_("Image file \"{0}\" cannot be saved. {1}"), filename, strerror(errno)) << endl;
        return false;
    }
    
    if(setjmp(png_jmpbuf(png_ptr))){
        png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
        fclose(fp);
        os << format(_("Internal error in saving \"{0}\"."), filename) << endl;
        return false;
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

    vector<png_bytep> row_pointers(height);
    for(int i=0; i < height; ++i){
        row_pointers[i] = const_cast<unsigned char*>(image.pixels()) + width * image.numComponents() * i;
    }
    png_write_image(png_ptr, &row_pointers[0]);
    png_write_end(png_ptr, info_ptr);
    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
    fclose(fp);

    return true;
}

    
bool loadJPEG(Image& image, const std::string& filename, bool isUpsideDown, ostream& os)
{
    FILE* fp = fopen(fromUTF8(filename).c_str(), "rb");
    if(!fp){
        os << format(_("Image file \"{0}\" cannot be loaded. {1}"), filename, strerror(errno)) << endl;
        return false;
    }

    bool loaded = false;
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
        
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, fp);
        
    jpeg_read_header(&cinfo, TRUE);
    jpeg_start_decompress(&cinfo);
    image.setSize(cinfo.output_width, cinfo.output_height, cinfo.output_components);
        
    unsigned char* pixels = image.pixels();
    const int h = image.height();
    const int w = image.width();

    vector<JSAMPROW> row_pointers(image.height());
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
        jpeg_read_scanlines(
            &cinfo, &row_pointers[0] + cinfo.output_scanline, cinfo.output_height - cinfo.output_scanline);
    }
        
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(fp);

    return true;
}


bool loadTGA(Image& image, const std::string& filename, bool isUpsideDown, ostream& os)
{
    FILE* fp = fopen(filename.c_str(), "rb");
    if(!fp){
        os << format(_("Image file \"{0}\" cannot be loaded. {1}"), filename, strerror(errno)) << endl;
        return false;
    }

    unsigned char header[12]={0,0,2,0,0,0,0,0,0,0,0,0};
    unsigned char header_buf[12];
    unsigned char header_buf2[6];

    if( fread(header_buf,1,sizeof(header),fp)!=sizeof(header) ||
        memcmp(header,header_buf,sizeof(header))!=0 ||
        fread(header_buf2, 1, sizeof(header_buf2), fp)!=sizeof(header_buf2) ){
        fclose(fp);
        os << format(_("Image file \"{0}\" is not the uncompressed TGA format."), filename) << endl;
        return false;
    }

    unsigned int width = header_buf2[1] * 256 + header_buf2[0];
    unsigned int height = header_buf2[3] * 256 + header_buf2[2];
    unsigned int bitPerPixel = header_buf2[4];
    unsigned int bytesPerPixel = bitPerPixel/8;

    if( width<=0 || height<=0 || (bytesPerPixel!=3 && bytesPerPixel!=4) )
    {
        fclose(fp);
        image.setSize(0, 0);
        os << format(_("Image file \"{0}\" is empty."), filename) << endl;
        return false;
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
                os << format(_("Internal error in loading \"{0}\"."), filename) << endl;
                return false;
            }

            pixels[k++] = imageBuf[2];
            pixels[k++] = imageBuf[1];
            pixels[k++] = imageBuf[0];
            if(bytesPerPixel==4)
                pixels[k++] = imageBuf[3];
        }
    }

    fclose (fp);
    
    return true;
}

}


ImageIO::ImageIO()
{
    isUpsideDown_ = false;
}


bool ImageIO::load(Image& image, const std::string& filename, std::ostream& os)
{
    bool loaded = false;
    
    filesystem::path fpath(filename);
    string ext = fpath.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    
    if(ext == ".png"){
        loaded = loadPNG(image, filename, isUpsideDown_, os);
    } else if(ext == ".jpg" || ext == ".jpeg"){
        loaded = loadJPEG(image, filename, isUpsideDown_, os);
    } else if(ext == ".tga"){
        loaded = loadTGA(image, filename, isUpsideDown_, os);
    } else {
        os << format(_("The image file format of \"{0}\" is not supported."), filename) << endl;
    }

    return loaded;
}


bool ImageIO::save(const Image& image, const std::string& filename, std::ostream& os)
{
    bool saved = false;
    
    filesystem::path fpath(filename);
    string ext = fpath.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    
    if(ext == ".png"){
        saved = savePNG(image, filename, isUpsideDown_, os);
    } else {
        os << format(_("The image file format of \"{0}\" is not supported."), filename) << endl;
    }

    return saved;
}
