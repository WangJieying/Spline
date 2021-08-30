// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2020
// Distributed under the Boost Software License, Version 1.0.
// https://www.boost.org/LICENSE_1_0.txt
// https://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// Version: 4.0.2019.08.13

#include <Applications/GTApplicationsPCH.h>
#include <Applications/Environment.h>
#include <Applications/MSW/WICFileIO.h>
#include <Mathematics/Logger.h>
#include <Mathematics/StringUtility.h>

// For access to the reference-counting COM interface wrapper.  The header
// wrl.h includes windows.h, so we must turn off min and max macros.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <wincodec.h>
#include <wrl.h>
using namespace gte;

class ComInitializer
{
public:
    ~ComInitializer()
    {
        if (mInitialized)
        {
            ::CoUninitialize();
        }
    }

    ComInitializer()
    {
        HRESULT hr = ::CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
        mInitialized = SUCCEEDED(hr);
    }

    inline bool IsInitialized() const
    {
        return mInitialized;
    }

private:
    bool mInitialized;
};

template <typename T>
using ComObject = Microsoft::WRL::ComPtr<T>;

std::shared_ptr<Texture2> WICFileIO::Load(std::string const& filename, bool wantMipmaps)
{
    // Start COM and create WIC.
    ComInitializer comInitializer;
    if (!comInitializer.IsInitialized())
    {
        LogError("Unable to initialize COM for WIC.");
    }

    // Create a WIC imaging factory.
    ComObject<IWICImagingFactory> wicFactory;
    HRESULT hr = ::CoCreateInstance(CLSID_WICImagingFactory, nullptr,
        CLSCTX_INPROC_SERVER, IID_IWICImagingFactory,
        reinterpret_cast<LPVOID*>(wicFactory.GetAddressOf()));
    if (FAILED(hr))
    {
        LogError("Unable to create WIC imaging factory.");
    }

    // Create a decoder based on the file name.
    std::wstring wfilename(filename.begin(), filename.end());
    ComObject<IWICBitmapDecoder> wicDecoder;
    hr = wicFactory->CreateDecoderFromFilename(wfilename.c_str(),
        nullptr, GENERIC_READ, WICDecodeMetadataCacheOnDemand, &wicDecoder);
    if (FAILED(hr))
    {
        LogError("wicFactory->CreateDecoderFromFilename failed (" + filename + ").");
    }

    // Create a WIC decoder.
    ComObject<IWICBitmapFrameDecode> wicFrameDecode;
    hr = wicDecoder->GetFrame(0, &wicFrameDecode);
    if (FAILED(hr))
    {
        LogError("wicDecoder->GetFrame failed.");
    }

    // Get the pixel format of the image.
    WICPixelFormatGUID wicSourceGUID;
    hr = wicFrameDecode->GetPixelFormat(&wicSourceGUID);
    if (FAILED(hr))
    {
        LogError("wicFrameDecode->GetPixelFormat failed.");
    }

    // Find the supported WIC input pixel format that matches a Texture2
    // format.  If a matching format is not found, the returned texture
    // is an R8G8B8A8 format with texels converted from the source format.
    WICPixelFormatGUID wicConvertGUID = GUID_WICPixelFormat32bppRGBA;
    DFType gtformat = DF_R8G8B8A8_UNORM;
    for (int i = 0; i < NUM_LOAD_FORMATS; ++i)
    {
        if (IsEqualGUID(wicSourceGUID, *msLoadFormatMap[i].wicInputGUID))
        {
            // Determine whether there is a conversion format.
            if (msLoadFormatMap[i].wicConvertGUID)
            {
                wicConvertGUID = *msLoadFormatMap[i].wicConvertGUID;
            }
            else
            {
                wicConvertGUID = *msLoadFormatMap[i].wicInputGUID;
            }
            gtformat = msLoadFormatMap[i].gtFormat;
            break;
        }
    }

    // The wicFrameDecode value is used for no conversion.  If the decoder
    // does not support the format in the texture, then a conversion is
    // required.
    IWICBitmapSource* wicBitmapSource = wicFrameDecode.Get();
    ComObject<IWICFormatConverter> wicFormatConverter;
    if (!IsEqualGUID(wicSourceGUID, wicConvertGUID))
    {
        // Create a WIC format converter.
        hr = wicFactory->CreateFormatConverter(&wicFormatConverter);
        if (FAILED(hr))
        {
            LogError("wicFactory->CreateFormatConverter failed.");
        }

        // Initialize format converter to convert the input texture format
        // to the nearest format supported by the decoder.
        hr = wicFormatConverter->Initialize(wicFrameDecode.Get(), wicConvertGUID,
            WICBitmapDitherTypeNone, nullptr, 0.0,
            WICBitmapPaletteTypeCustom);
        if (FAILED(hr))
        {
            LogError("wicFormatConverter->Initialize failed.");
        }

        // Use the format converter.
        wicBitmapSource = wicFormatConverter.Get();
    }

    // Get the image dimensions.
    UINT width, height;
    hr = wicBitmapSource->GetSize(&width, &height);
    if (FAILED(hr))
    {
        LogError("wicBitmapSource->GetSize failed.");
    }

    // Create the 2D texture and compute the stride and image size.
    std::shared_ptr<Texture2> texture = std::make_shared<Texture2>(
        gtformat, width, height, wantMipmaps);
    UINT const stride = width * texture->GetElementSize();
    UINT const imageSize = stride * height;

    // Copy the pixels from the decoder to the texture.
    hr = wicBitmapSource->CopyPixels(nullptr, stride, imageSize,
        texture->Get<BYTE>());
    if (FAILED(hr))
    {
        LogError("wicBitmapSource->CopyPixels failed.");
    }

    return texture;
}

std::shared_ptr<Texture2> WICFileIO::Load(void* module, std::string const& rtype,
    int resource, bool wantMipmaps)
{
    auto hModule = reinterpret_cast<HMODULE>(module);

    // Start COM and create WIC.
    ComInitializer comInitializer;
    if (!comInitializer.IsInitialized())
    {
        LogError("Unable to initialize COM for WIC.");
    }

    // Create a WIC imaging factory.
    ComObject<IWICImagingFactory> wicFactory;
    HRESULT hr = ::CoCreateInstance(CLSID_WICImagingFactory, nullptr,
        CLSCTX_INPROC_SERVER, IID_IWICImagingFactory,
        reinterpret_cast<LPVOID*>(wicFactory.GetAddressOf()));
    if (FAILED(hr))
    {
        LogError("Unable to create WIC imaging factory.");
    }

    // WIC interface pointers.
    IWICStream* pIWICStream = NULL;
    IWICBitmapDecoder* pIDecoder = NULL;
    IWICBitmapFrameDecode* pIDecoderFrame = NULL;

    // Resource management.
    HRSRC imageResHandle = NULL;
    HGLOBAL imageResDataHandle = NULL;
    void* pImageFile = NULL;
    DWORD imageFileSize = 0;

    // Locate the resource in the application's executable.
    auto const wrtype = ConvertNarrowToWide(rtype);
    imageResHandle = FindResource(hModule, MAKEINTRESOURCE(resource), wrtype.c_str());
    if (!imageResHandle)
    {
        LogError("FindResource failed.");
    }

    imageResDataHandle = LoadResource(hModule, imageResHandle);
    if (!imageResDataHandle)
    {
        LogError("LoadResource failed.");
    }

    // Lock the resource to retrieve memory pointer.
    pImageFile = LockResource(imageResDataHandle);
    if (!pImageFile)
    {
        LogError("LockResource failed.");
    }

    // Calculate the size.
    imageFileSize = SizeofResource(hModule, imageResHandle);
    if (!imageFileSize)
    {
        LogError("SizeofResource failed.");
    }

    // Create a WIC stream to map onto the memory.
    hr = wicFactory->CreateStream(&pIWICStream);
    if (FAILED(hr))
    {
        LogError("wicFactory->CreateStream failed.");
    }

    // Initialize the stream with the memory pointer and size.
    hr = pIWICStream->InitializeFromMemory(
        reinterpret_cast<BYTE*>(pImageFile),
        imageFileSize);
    if (FAILED(hr))
    {
        LogError("wicFactory->InitializeFromMemory failed.");
    }

    // Create a decoder for the stream.
    hr = wicFactory->CreateDecoderFromStream(
        pIWICStream,                   // stream for creating the decoder
        NULL,                          // do not prefer a particular vendor
        WICDecodeMetadataCacheOnLoad,  // cache metadata when needed
        &pIDecoder);                   // pointer to the decoder
    if (FAILED(hr))
    {
        LogError("wicFactory->CreateDecoderFromStream failed.");
    }

    hr = pIDecoder->GetFrame(0, &pIDecoderFrame);
    if (FAILED(hr))
    {
        LogError("pIDecoder->GetFrame failed.");
    }

    // Get the pixel format of the image.
    WICPixelFormatGUID wicSourceGUID;
    hr = pIDecoderFrame->GetPixelFormat(&wicSourceGUID);
    if (FAILED(hr))
    {
        LogError("wicFrameDecode->GetPixelFormat failed.");
    }

    // Find the supported WIC input pixel format that matches a Texture2
    // format.  If a matching format is not found, the returned texture
    // is an R8G8B8A8 format with texels converted from the source format.
    WICPixelFormatGUID wicConvertGUID = GUID_WICPixelFormat32bppRGBA;
    DFType gtformat = DF_R8G8B8A8_UNORM;
    for (int i = 0; i < NUM_LOAD_FORMATS; ++i)
    {
        if (IsEqualGUID(wicSourceGUID, *msLoadFormatMap[i].wicInputGUID))
        {
            // Determine whether there is a conversion format.
            if (msLoadFormatMap[i].wicConvertGUID)
            {
                wicConvertGUID = *msLoadFormatMap[i].wicConvertGUID;
            }
            else
            {
                wicConvertGUID = *msLoadFormatMap[i].wicInputGUID;
            }
            gtformat = msLoadFormatMap[i].gtFormat;
            break;
        }
    }

    // The wicFrameDecode value is used for no conversion.  If the decoder
    // does not support the format in the texture, then a conversion is
    // required.
    IWICBitmapSource* wicBitmapSource = pIDecoderFrame;
    ComObject<IWICFormatConverter> wicFormatConverter;
    if (!IsEqualGUID(wicSourceGUID, wicConvertGUID))
    {
        // Create a WIC format converter.
        hr = wicFactory->CreateFormatConverter(&wicFormatConverter);
        if (FAILED(hr))
        {
            LogError("wicFactory->CreateFormatConverter failed.");
        }

        // Initialize format converter to convert the input texture format
        // to the nearest format supported by the decoder.
        hr = wicFormatConverter->Initialize(pIDecoderFrame, wicConvertGUID,
            WICBitmapDitherTypeNone, nullptr, 0.0,
            WICBitmapPaletteTypeCustom);
        if (FAILED(hr))
        {
            LogError("wicFormatConverter->Initialize failed.");
        }

        // Use the format converter.
        wicBitmapSource = wicFormatConverter.Get();
    }

    // Get the image dimensions.
    UINT width, height;
    hr = wicBitmapSource->GetSize(&width, &height);
    if (FAILED(hr))
    {
        LogError("wicBitmapSource->GetSize failed.");
    }

    // Create the 2D texture and compute the stride and image size.
    auto texture = std::make_shared<Texture2>(gtformat, width, height, wantMipmaps);
    UINT const stride = width * texture->GetElementSize();
    UINT const imageSize = stride * height;

    // Copy the pixels from the decoder to the texture.
    hr = wicBitmapSource->CopyPixels(nullptr, stride, imageSize, texture->Get<BYTE>());
    if (FAILED(hr))
    {
        LogError("wicBitmapSource->CopyPixels failed.");
    }

    return texture;
}

bool WICFileIO::SaveToPNG(std::string const& filename,
    std::shared_ptr<Texture2> const& texture)
{
    return SaveTo(filename, texture, -1.0f);
}

bool WICFileIO::SaveToJPEG(std::string const& filename,
    std::shared_ptr<Texture2> const& texture, float imageQuality)
{
    imageQuality = std::min(std::max(imageQuality, 0.0f), 1.0f);
    return SaveTo(filename, texture, imageQuality);
}

bool WICFileIO::SaveTo(std::string const& filename,
    std::shared_ptr<Texture2> const& texture, float imageQuality)
{
    if (!texture || !texture->GetData())
    {
        LogError("The texture and its data must exist.");
    }

    // Select the WIC format that matches the input texture format.
    WICPixelFormatGUID wicSourceGUID = GUID_WICPixelFormatUndefined;
    for (int i = 0; i < NUM_SAVE_FORMATS; ++i)
    {
        if (msSaveFormatMap[i].gtFormat == texture->GetFormat())
        {
            wicSourceGUID = *msSaveFormatMap[i].wicOutputGUID;
            break;
        }
    }
    if (IsEqualGUID(wicSourceGUID, GUID_WICPixelFormatUndefined))
    {
        LogError("Format " +
            DataFormat::GetName(texture->GetFormat()) +
            "is not supported for saving.");
    }

    // Start COM and create WIC.
    ComInitializer comInitializer;
    if (!comInitializer.IsInitialized())
    {
        LogError("Unable to initialize COM for WIC.");
    }

    // Create a WIC imaging factory.
    ComObject<IWICImagingFactory> wicFactory;
    HRESULT hr = ::CoCreateInstance(CLSID_WICImagingFactory, nullptr,
        CLSCTX_INPROC_SERVER, IID_IWICImagingFactory,
        reinterpret_cast<LPVOID*>(wicFactory.GetAddressOf()));
    if (FAILED(hr))
    {
        LogError("Unable to create WIC imaging factory.");
    }

    // Create a WIC stream for output.
    ComObject<IWICStream> wicStream;
    hr = wicFactory->CreateStream(&wicStream);
    if (FAILED(hr))
    {
        LogError("wicFactory->CreateStream failed.");
    }

    std::wstring wfilename(filename.begin(), filename.end());
    hr = wicStream->InitializeFromFilename(wfilename.c_str(), GENERIC_WRITE);
    if (FAILED(hr))
    {
        LogError("wicStream->InitializeFromFilename failed (" +
            filename + ").");
    }

    // Create a WIC JPEG encoder.
    ComObject<IWICBitmapEncoder> wicEncoder;
    if (imageQuality == -1.0f)
    {
        hr = wicFactory->CreateEncoder(GUID_ContainerFormatPng, nullptr, &wicEncoder);
    }
    else
    {
        hr = wicFactory->CreateEncoder(GUID_ContainerFormatJpeg, nullptr, &wicEncoder);
    }
    if (FAILED(hr))
    {
        LogError("wicFactory->CreateEncoder failed.");
    }

    hr = wicEncoder->Initialize(wicStream.Get(), WICBitmapEncoderNoCache);
    if (FAILED(hr))
    {
        LogError("wicEncoder->Initialize failed.");
    }

    // Create a new frame and a property bag for encoder options.
    ComObject<IWICBitmapFrameEncode> wicFrameEncode;
    ComObject<IPropertyBag2> wicPropertyBag;
    hr = wicEncoder->CreateNewFrame(&wicFrameEncode, &wicPropertyBag);
    if (FAILED(hr))
    {
        LogError("wicEncoder->CreateNewFrame failed.");
    }

    if (imageQuality == -1.0f)
    {
        // Set the options for the PNG encoder.
        PROPBAG2 option = { 0 };
        VARIANT varValue;

        // Default subsampling.
        option.pstrName = const_cast<LPOLESTR>(L"InterlaceOption");
        VariantInit(&varValue);
        varValue.vt = VT_BOOL;
        varValue.boolVal = FALSE;
        hr = wicPropertyBag->Write(1, &option, &varValue);
        if (FAILED(hr))
        {
            LogError("wicPropertyBag->Write failed for InterlaceOption.");
        }

        // Disable filtering.
        option.pstrName = const_cast<LPOLESTR>(L"FilterOption");
        VariantInit(&varValue);
        varValue.vt = VT_UI1;
#if defined(MINGW)
        // The wincodec.h of MinGW 7.2.0 does not define the
        // WICPngFilterOption enumerations.
        varValue.bVal = 1;
#else
        varValue.bVal = WICPngFilterNone;
#endif
        hr = wicPropertyBag->Write(1, &option, &varValue);
        if (FAILED(hr))
        {
            LogError("wicPropertyBag->Write failed for FilterOption.");
        }
    }
    else
    {
        // Set the options for the PNG encoder.
        PROPBAG2 option = { 0 };
        VARIANT varValue;

        // Set image quality, a number in [0,1].
        option.pstrName = const_cast<LPOLESTR>(L"ImageQuality");
        VariantInit(&varValue);
        varValue.vt = VT_R4;
        varValue.fltVal = imageQuality;
        hr = wicPropertyBag->Write(1, &option, &varValue);
        if (FAILED(hr))
        {
            LogError("wicPropertyBag->Write failed for ImageQuality.");
        }
    }

    // Initialize the encoder.
    hr = wicFrameEncode->Initialize(wicPropertyBag.Get());
    if (FAILED(hr))
    {
        LogError("wicFrameEncode->Initialize failed.");
    }

    // Set the image size.
    UINT width = texture->GetWidth();
    UINT height = texture->GetHeight();
    hr = wicFrameEncode->SetSize(width, height);
    if (FAILED(hr))
    {
        LogError("wicFrameEncode->SetSize failed.");
    }

    // Set the image format.
    WICPixelFormatGUID wicTargetGUID = wicSourceGUID;
    hr = wicFrameEncode->SetPixelFormat(&wicTargetGUID);
    if (FAILED(hr))
    {
        LogError("wicFrameEncode->SetPixelFormat failed.");
    }

    // Compute the stride and image size.
    UINT const stride = width * texture->GetElementSize();
    UINT const imageSize = stride * height;

    // Create a WIC bitmap to wrap the texture image data.
    ComObject<IWICBitmap> wicTextureBitmap;
    hr = wicFactory->CreateBitmapFromMemory(width, height,
        wicSourceGUID, stride, imageSize, texture->Get<BYTE>(),
        &wicTextureBitmap);
    if (FAILED(hr))
    {
        LogError("wicFactory->CreateBitmapFromMemory failed.");
    }

    // The wicTextureBitmap value is used for no conversion.  If the encoder
    // does not support the format in the texture, then a conversion is
    // required.
    IWICBitmapSource* wicBitmapSource = wicTextureBitmap.Get();
    ComObject<IWICFormatConverter> wicFormatConverter;
    if (!IsEqualGUID(wicSourceGUID, wicTargetGUID))
    {
        // Create a WIC format converter.
        hr = wicFactory->CreateFormatConverter(&wicFormatConverter);
        if (FAILED(hr))
        {
            LogError("wicFactory->CreateFormatConverter failed.");
        }

        // Initialize the format converter to convert to the nearest format
        // supported by the encoder.
        hr = wicFormatConverter->Initialize(wicTextureBitmap.Get(), wicTargetGUID,
            WICBitmapDitherTypeNone, nullptr, 0.0, WICBitmapPaletteTypeCustom);
        if (FAILED(hr))
        {
            LogError("wicFormatConverter->Initialize failed.");
        }

        // Use the format converter.
        wicBitmapSource = wicFormatConverter.Get();
    }

    // Send the pixels to the encoder.
    hr = wicFrameEncode->WriteSource(wicBitmapSource, nullptr);
    if (FAILED(hr))
    {
        LogError("wicFrameEncode->WriteSource failed.");
    }

    // Commit the frame.
    hr = wicFrameEncode->Commit();
    if (FAILED(hr))
    {
        LogError("wicFrameEncode->Commit failed.");
    }

    // Commit the encoder.
    hr = wicEncoder->Commit();
    if (FAILED(hr))
    {
        LogError("wicEncoder->Commit failed.");
    }

    return true;
}

WICFileIO::LoadFormatMap const WICFileIO::msLoadFormatMap[NUM_LOAD_FORMATS] =
{
    { DF_R10G10B10A2_UNORM, &GUID_WICPixelFormat32bppRGBA1010102, nullptr },
    { DF_R10G10B10_XR_BIAS_A2_UNORM, &GUID_WICPixelFormat32bppRGBA1010102XR, nullptr },
    { DF_R32_FLOAT, &GUID_WICPixelFormat32bppGrayFloat, nullptr },
    { DF_R16G16B16A16_UNORM, &GUID_WICPixelFormat64bppBGRA, &GUID_WICPixelFormat64bppRGBA },
    { DF_B5G6R5_UNORM, &GUID_WICPixelFormat16bppBGR565, nullptr },
    { DF_B5G5R5A1_UNORM, &GUID_WICPixelFormat16bppBGR555, nullptr },
    { DF_R1_UNORM, &GUID_WICPixelFormatBlackWhite, &GUID_WICPixelFormat8bppGray },
    { DF_R8_UNORM, &GUID_WICPixelFormat2bppGray, &GUID_WICPixelFormat8bppGray },
    { DF_R8_UNORM, &GUID_WICPixelFormat4bppGray, &GUID_WICPixelFormat8bppGray },
    { DF_R8_UNORM, &GUID_WICPixelFormat8bppGray, nullptr },
    { DF_R16_UNORM, &GUID_WICPixelFormat16bppGray, nullptr },
    { DF_R8G8B8A8_UNORM, &GUID_WICPixelFormat32bppRGBA, nullptr },
    { DF_R8G8B8A8_UNORM, &GUID_WICPixelFormat32bppBGRA, &GUID_WICPixelFormat32bppRGBA },
    { DF_R16G16B16A16_UNORM, &GUID_WICPixelFormat64bppRGBA, nullptr },

    // B8G8R8A8 is not supported for Texture2 in DX11.  We convert all
    // unmatched formats to R8G8B8A8.
    //{ DF_B8G8R8A8_UNORM, &GUID_WICPixelFormat32bppBGRA }
};

WICFileIO::SaveFormatMap const WICFileIO::msSaveFormatMap[NUM_SAVE_FORMATS] =
{
    // The wincodec.h of MinGW 7.2.0 does not support these formats.
    { DF_R10G10B10A2_UNORM, &GUID_WICPixelFormat32bppRGBA1010102 },
    { DF_R10G10B10_XR_BIAS_A2_UNORM, &GUID_WICPixelFormat32bppRGBA1010102XR },
    { DF_R32_FLOAT, &GUID_WICPixelFormat32bppGrayFloat },
    { DF_B5G6R5_UNORM, &GUID_WICPixelFormat16bppBGR565 },
    { DF_B5G5R5A1_UNORM, &GUID_WICPixelFormat16bppBGR555 },
    { DF_R1_UNORM, &GUID_WICPixelFormatBlackWhite },
    { DF_R8_UNORM, &GUID_WICPixelFormat8bppGray },
    { DF_R16_UNORM, &GUID_WICPixelFormat16bppGray },
    { DF_R8G8B8A8_UNORM, &GUID_WICPixelFormat32bppRGBA },
    { DF_B8G8R8A8_UNORM, &GUID_WICPixelFormat32bppBGRA },
    { DF_R16G16B16A16_UNORM, &GUID_WICPixelFormat64bppRGBA }
};
