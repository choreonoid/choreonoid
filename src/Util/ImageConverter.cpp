/**
   @file ImageConverter.cpp
   @brief Implementation of Image Converter class
   @author K.FUKUDA
   @author Shin'ichiro Nakaoka
*/

#include "ImageConverter.h"
#include <algorithm>
#include <iostream>

extern "C" {
#define XMD_H
#include <jpeglib.h>
}
#include <png.h>

using namespace std;
using namespace cnoid;


ImageConverter::ImageConverter(void)
{
    image = new SFImage;
}


ImageConverter::~ImageConverter(void)
{
    delete image;
}

 
/**
   @brief      initialize "SFImage"
   @note       use before using "SFImage" structure 
   @return     bool true : succeeded / false : failed
*/
bool ImageConverter::initializeSFImage( )
{
    image->width = 0;
    image->height = 0;
    image->numComponents = 0;
    image->pixels.clear();

    return true;
}


/**
   @brief      convert ImageTexture node to PixelTexture node
   @note       read image data from VrmlImageTexture::url and store pixel data to
   VrmlPixelTexture::image.
   *.png and *.jpg are supported.
   Currentry, multi url is not supported.
   @return     bool true : succeeded / false : failed
*/
SFImage* ImageConverter::convert(const std::string& url)
{
    message.clear();
    string  ext = url.substr( url.rfind( '.' ) );

    // convert the ext name into lower letters
    transform( ext.begin(), ext.end(), ext.begin(), (int(*)(int))tolower );

    if( !ext.compare( ".png" ) )
        {
            if(loadPNG( url ))
                return image;
        }
    else if( !ext.compare( ".jpg" ) )
        {
            if(loadJPEG( url ))
                return image;
        }
    else
        {
            message += "ImageTexture read error: \"" + ext + "\"  is unsupported format.";
            // cerr << "ImageTexture read error: unsupported format." << '\n';
        }

    image->height = 0;
    image->width = 0;
    image->numComponents = 0;
    image->pixels.resize(0);
    return image;
}


/**
   @brief      load PNG file
   @note       load and fill VrmlPixelTexture::image from PNG.
   @return     bool true : succeeded / false : failed
*/
bool ImageConverter::loadPNG(const std::string& filePath)
{
    initializeSFImage( );

    FILE* fp = 0;

    try
        {
            fp = fopen(filePath.c_str(), "rb");
            if(!fp){
                throw "File open error.";
            }

            png_size_t  number = 8;
            png_byte    header[8];
            int         is_png;

            size_t n = fread(header, 1, number, fp);
            if(n != number){
                throw "File is not png.";
            }
            is_png = !png_sig_cmp(header, 0, number);
            if(!is_png){
                throw "File is not png.";
            }

            png_structp pPng;

            pPng = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
            if(!pPng){
                throw "Failed to create png_struct";
            }

            png_infop   pInfo;
        
            pInfo = png_create_info_struct(pPng);
            if(!pInfo){
                png_destroy_read_struct( &pPng, NULL, NULL );
                throw "Failed to create png_info";
            }

            png_init_io(pPng, fp);
            png_set_sig_bytes( pPng, (int)number );

            png_read_info(pPng, pInfo);
            png_uint_32 width   = png_get_image_width( pPng, pInfo );
            png_uint_32 height  = png_get_image_height( pPng, pInfo );
            png_byte color_type = png_get_color_type( pPng, pInfo );
            png_byte depth = png_get_bit_depth( pPng, pInfo );
        
            if (png_get_valid( pPng, pInfo, PNG_INFO_tRNS)) 
                png_set_tRNS_to_alpha(pPng);
            if (depth < 8)
                png_set_packing(pPng);
        
            int numComponents;
            switch( color_type )
                {
                case PNG_COLOR_TYPE_GRAY:
                    numComponents = 1;
                    if(depth < 8) 
                        png_set_expand_gray_1_2_4_to_8(pPng);
                    break;

                case PNG_COLOR_TYPE_GRAY_ALPHA:
                    numComponents = 2;
                    if (depth == 16)
                        png_set_strip_16(pPng);
                    break;

                case PNG_COLOR_TYPE_RGB:
                    numComponents = 3;
                    if (depth == 16)
                        png_set_strip_16(pPng);
                    break;
            
                case PNG_COLOR_TYPE_RGB_ALPHA:
                    numComponents = 4;
                    if (depth == 16)
                        png_set_strip_16(pPng);
                    break;

                case PNG_COLOR_TYPE_PALETTE:
                    png_set_palette_to_rgb(pPng);
                    numComponents = 3;
                    break;

                default:
                    numComponents = 1;
                    //  throw "Unsupported color type.";

                }

            png_read_update_info( pPng, pInfo );        

            unsigned char** row_pointers;
            row_pointers = (png_bytepp)malloc(height * sizeof(png_bytep)); 
            png_uint_32 rowbytes = png_get_rowbytes(pPng, pInfo);
            image->pixels.resize(rowbytes*height);
            for(png_uint_32 i=0; i < height; i++){
                row_pointers[i] = &(image->pixels[i*rowbytes]);
            }
            png_read_image(pPng, row_pointers);  

            image->width = width;
            image->height = height;
            image->numComponents = numComponents;
                  
            free(row_pointers);
            png_destroy_read_struct( &pPng, &pInfo, NULL );

            fclose(fp);
        }

    catch( char * str )
        {
            message += "PNG read error: " + string(str) + '\n';
            if(fp) fclose(fp);
            return false;
        }

    return true;
}


/**
   @brief      load JPEG file
   @note       load and fill VrmlPixelTexture::image from JPEG.
   @return     bool true : succeeded / false : failed
*/
bool ImageConverter::loadJPEG(const std::string& filePath)
{
    initializeSFImage( );

    FILE* fp = 0;

    try
        {
            // File open
            fp = fopen(filePath.c_str(), "rb");
            if( !fp )       throw "File open error.";


            struct jpeg_decompress_struct   cinfo;
            struct jpeg_error_mgr           jerr;

            // Step 1: allocate and initialize JPEG decompression object
            cinfo.err = jpeg_std_error( &jerr );
            jpeg_create_decompress( &cinfo );


            // Step 2: specify data source (eg, a file)
            jpeg_stdio_src( &cinfo, fp );

            // Step 3: read file parameters with jpeg_read_header()
            (void)jpeg_read_header( &cinfo, TRUE );


            // Step 4: set parameters for decompression


            // Step 5: Start decompression
            (void)jpeg_start_decompress( &cinfo );


            // get image attribute
            image->width         = cinfo.output_width;
            image->height        = cinfo.output_height;
            image->numComponents = cinfo.out_color_components;
       
            JSAMPARRAY row_pointers;
            row_pointers = (JSAMPARRAY)malloc( sizeof( JSAMPROW ) * image->height );
            image->pixels.resize(cinfo.output_components * image->width * image->height);
            for (int i = 0; i < image->height; i++ ) 
                row_pointers[i] = &(image->pixels[i * cinfo.output_components * image->width]);

            while( cinfo.output_scanline < cinfo.output_height ) {
                jpeg_read_scanlines( &cinfo,
                                     row_pointers + cinfo.output_scanline,
                                     cinfo.output_height - cinfo.output_scanline
                    );
            }

            free(row_pointers);

            // Step 7: Finish decompression
            (void)jpeg_finish_decompress( &cinfo );


            // Step 8: Release JPEG decompression object
            jpeg_destroy_decompress( &cinfo );


            fclose( fp );
        }

    catch( char * str )
        {
            message += "JPEG read error: " + string(str) + '\n';
            if( fp ) fclose( fp );
            return false;
        }

    return true;
}
