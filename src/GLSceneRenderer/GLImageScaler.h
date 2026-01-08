#ifndef CNOID_GL_SCENE_RENDERER_GL_IMAGE_SCALER_H
#define CNOID_GL_SCENE_RENDERER_GL_IMAGE_SCALER_H

namespace cnoid {

/**
   Scale an image using bilinear interpolation.
   This function is used to scale textures to power-of-two dimensions
   for OpenGL implementations that don't support NPOT textures.

   @param srcWidth Source image width
   @param srcHeight Source image height
   @param numComponents Number of color components per pixel (1-4)
   @param srcPixels Pointer to source pixel data
   @param dstWidth Destination image width
   @param dstHeight Destination image height
   @param dstPixels Pointer to destination pixel buffer (must be pre-allocated)
*/
inline void scaleImageBilinear(
    int srcWidth, int srcHeight, int numComponents,
    const unsigned char* srcPixels,
    int dstWidth, int dstHeight,
    unsigned char* dstPixels)
{
    const float xRatio = static_cast<float>(srcWidth) / dstWidth;
    const float yRatio = static_cast<float>(srcHeight) / dstHeight;
    const int srcStride = srcWidth * numComponents;
    const int dstStride = dstWidth * numComponents;

    for(int dstY = 0; dstY < dstHeight; ++dstY){
        const float srcYf = dstY * yRatio;
        const int srcY0 = static_cast<int>(srcYf);
        const int srcY1 = (srcY0 + 1 < srcHeight) ? srcY0 + 1 : srcY0;
        const float yFrac = srcYf - srcY0;
        const float yFrac1 = 1.0f - yFrac;

        unsigned char* dstRow = dstPixels + dstY * dstStride;

        for(int dstX = 0; dstX < dstWidth; ++dstX){
            const float srcXf = dstX * xRatio;
            const int srcX0 = static_cast<int>(srcXf);
            const int srcX1 = (srcX0 + 1 < srcWidth) ? srcX0 + 1 : srcX0;
            const float xFrac = srcXf - srcX0;
            const float xFrac1 = 1.0f - xFrac;

            // Get pointers to the four neighboring pixels
            const unsigned char* p00 = srcPixels + srcY0 * srcStride + srcX0 * numComponents;
            const unsigned char* p10 = srcPixels + srcY0 * srcStride + srcX1 * numComponents;
            const unsigned char* p01 = srcPixels + srcY1 * srcStride + srcX0 * numComponents;
            const unsigned char* p11 = srcPixels + srcY1 * srcStride + srcX1 * numComponents;

            // Bilinear interpolation weights
            const float w00 = xFrac1 * yFrac1;
            const float w10 = xFrac  * yFrac1;
            const float w01 = xFrac1 * yFrac;
            const float w11 = xFrac  * yFrac;

            unsigned char* dst = dstRow + dstX * numComponents;
            for(int c = 0; c < numComponents; ++c){
                float value = p00[c] * w00 + p10[c] * w10 + p01[c] * w01 + p11[c] * w11;
                dst[c] = static_cast<unsigned char>(value + 0.5f);
            }
        }
    }
}

}

#endif
