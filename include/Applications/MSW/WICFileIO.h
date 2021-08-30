// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2020
// Distributed under the Boost Software License, Version 1.0.
// https://www.boost.org/LICENSE_1_0.txt
// https://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// Version: 4.0.2019.08.13

#pragma once

#include <Graphics/Texture2.h>
#include <guiddef.h>
#include <memory>

// The WICFileIO class provides simple loading and saving operations for
// texture data.  The JPEG operations use lossy compression and the PNG
// file operations use lossless compression.  Although the PNG operations
// are typically used for 2D images, you can use this class to save data
// that has nothing to do with images.  It is also possible to store 3D
// images by tiling a 2D image with the slices.  After the tiling it is
// possible that not all 2D image elements are occupied, but you can use
// your knowledge of the original 3D data to ignore the unoccupied pixels.

namespace gte
{
    class WICFileIO
    {
    public:
        // Support for loading from BMP, GIF, ICON, JPEG, PNG, and TIFF.
        // The returned texture has a format that matches as close as possible
        // the format on disk.  If the load is not successful, the function
        // returns a null object.
        //
        // The supported DF_* formats for loading are the following with
        // the first four formats not supported when using MINGW.  The
        // corresponding input GUID_WICPixelFormat* flags that are attempted
        // are shown also in the list.  If a parenthesized GUID is listed,
        // that is a conversion GUID.  The DF_ and GUID_WICPixelFormat
        // prefixes are not shown for readability.
        //
        //   DF_* format                    GUID_WICPixelFormat*
        //   -----------------------------------------------------------------
        //   R10G10B10A2_UNORM              32bppRGBA1010102
        //   R10G10B10_XR_BIAS_A2_UNORM     32bppRGBA1010102XR
        //   R32_FLOAT                      32bppGrayFloat
        //   R16G16B16A16_UNORM             64bppBGRA (64bppRGBA)
        //   B5G6R5_UNORM                   16bppBGR565
        //   B5G5R5A1_UNORM                 16bppBGR555
        //   R1_UNORM                       BlackWhite (8bppGray)
        //   R8_UNORM                       2bppGray (8bppGray)
        //   R8_UNORM                       4bppGray (8bppGray)
        //   R8_UNORM                       8ppGray
        //   R16_UNORM                      16bppGray
        //   R8G8B8A8_UNORM                 32bppRGBA
        //   R8G8B8A8_UNORM                 32bppBGRA (32bppRGBA)
        //   R16G16B16A16_UNORM             64bppRGBA
        static std::shared_ptr<Texture2> Load(std::string const& filename,
            bool wantMipmaps);

        // The resource-based loader takes the module parameter as a void*
        // and casts it back to HMODULE internally to avoid having to include
        // a bunch of Windows headers.
        static std::shared_ptr<Texture2> Load(void* module, std::string const& rtype,
            int resource, bool wantMipmaps);

        // Support for saving to PNG or JPEG.
        //
        // The supported DF_* formats for saving are the following with
        // the first three formats not supported when using MINGW.  The
        // corresponding GUID_WICPixelFormat* flags that are attempted are
        // shown also in the list.  The DF_ and GUID_WICPixelFormat prefixes
        // are not shown for readability.
        //
        //   DF_* format                    GUID_WICPixelFormat*
        //   -----------------------------------------------------------------
        //   R10G10B10A2_UNORM              32bppRGBA1010102
        //   R10G10B10_XR_BIAS_A2_UNORM     32bppRGBA1010102XR
        //   R32_FLOAT                      32bppGrayFloat
        //   B5G6R5_UNORM                   16bppBGR565
        //   B5G5R5A1_UNORM                 16bppBGR555
        //   R1_UNORM                       BlackWhite
        //   R8_UNORM                       8bppGray
        //   R16_UNORM                      16bppGray
        //   R8G8B8A8_UNORM                 32bppRGBA
        //   B8G8R8A8_UNORM                 32bppBGRA
        //   R16G16B16A16_UNORM             64bppRGBA

        static bool SaveToPNG(std::string const& filename,
            std::shared_ptr<Texture2> const& texture);

        // The image quality is in [0,1], where a value of 0 indicates lowest
        // quality (largest amount of compression) and a value of 1 indicates
        // highest quality (smallest amount of compression).
        static bool SaveToJPEG(std::string const& filename,
            std::shared_ptr<Texture2> const& texture, float imageQuality);

    private:
        struct LoadFormatMap
        {
            DFType gtFormat;
            GUID const* wicInputGUID;
            GUID const* wicConvertGUID;
        };
        enum { NUM_LOAD_FORMATS = 14 };
        static LoadFormatMap const msLoadFormatMap[NUM_LOAD_FORMATS];

        struct SaveFormatMap
        {
            DFType gtFormat;
            GUID const* wicOutputGUID;
        };
        enum { NUM_SAVE_FORMATS = 11 };
        static SaveFormatMap const msSaveFormatMap[NUM_SAVE_FORMATS];

        // Helper function to share code between saving PNG and JPEG.  Set
        // imageQuality to -1.0f for PNG.  Set it to a number in [0,1] for
        // JPEG.
        static bool SaveTo(std::string const& filename,
            std::shared_ptr<Texture2> const& texture, float imageQuality);
    };
}
