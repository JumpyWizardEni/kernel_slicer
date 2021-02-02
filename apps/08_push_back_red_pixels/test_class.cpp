#include "test_class.h"
#include "Bitmap.h"
#include <cassert>

inline bool PixelIsRed(uint32_t a_pixelValue)
{
  const uint32_t red   = (a_pixelValue & 0x000000FF);
  const uint32_t green = (a_pixelValue & 0x0000FF00) >> 8;
  const uint32_t blue  = (a_pixelValue & 0x00FF0000) >> 16;
  return (red >= 250) && (green < 5) && (blue < 5);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RedPixels::SetMaxDataSize(size_t a_size)
{
  m_size         = uint32_t(a_size);
  m_redPixelsNum = 0;
  m_foundPixels.reserve(a_size);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RedPixels::kernel1D_CountRedPixels(const uint32_t* a_data, size_t a_dataSize)
{
  m_redPixelsNum = 0;
  for(uint32_t i = 0; i<a_dataSize; i++)
  {
    if(PixelIsRed(a_data[i]))
      m_redPixelsNum++;
  }
}

void RedPixels::kernel1D_FindRedPixels(const uint32_t* a_data, size_t a_dataSize)
{
  m_foundPixels.resize(0);
  for(uint32_t i = 0; i<a_dataSize; i++)
  {
    const uint32_t pixValue = a_data[i];
    if(PixelIsRed(pixValue))
    {
      struct PixelInfo info;
      info.index = i;
      info.value = pixValue;
      m_foundPixels.push_back(info);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RedPixels::ProcessPixels(const uint32_t* a_data, size_t a_dataSize)
{
  kernel1D_CountRedPixels(a_data, a_dataSize);
  kernel1D_FindRedPixels (a_data, a_dataSize);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void process_image_cpu(const std::vector<uint32_t>& a_inPixels, std::vector<RedPixels::PixelInfo>& a_outPixels)
{
  RedPixels filter;

  filter.SetMaxDataSize(a_inPixels.size());
  filter.ProcessPixels(a_inPixels.data(), a_inPixels.size());
  a_outPixels = filter.GetFoundPixels();
  assert(a_outPixels.size() == filter.GetRedPixelsAmount());
  return;
}