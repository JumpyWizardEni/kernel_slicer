#include "test_class.h"
#include "Bitmap.h"
#include <cassert>
#include <algorithm>

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
  //m_pixelsCopy.resize(a_size);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void RedPixels::kernel1D_CountRedPixels(const uint32_t* a_data, size_t a_dataSize)
{
  m_redPixelsNum     = 0;
  m_otherPixelsNum   = 0;
  m_testPixelsAmount = 0.0f;
  m_testMin          = +100000000.0f;
  m_testMax          = -100000000.0f;

  for(uint32_t i = 0; i<a_dataSize; i++)
  {
    if(PixelIsRed(a_data[i]))
    {
      m_redPixelsNum++;
      m_testPixelsAmount -= 0.5f; // for example some function which define how pixel is red more precicely
    }
    else
    {
      float strangeVal = ((float)a_data[i])*((float)i);
      if(i == 1000)
        strangeVal = -10.0f;
      m_testMin = std::min(m_testMin, strangeVal);
      m_testMax = std::max(m_testMax, strangeVal);
      m_otherPixelsNum++;
    }
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
      PixelInfo info;
      info.index = i;
      info.value = pixValue;
      m_foundPixels.push_back(info);
    }
  }
}

void RedPixels::kernel1D_PaintRedPixelsToYellow(uint32_t* a_data)
{
  for(uint32_t i = 0; i<m_foundPixels.size(); i++)
    a_data[m_foundPixels[i].index] = 0x0000FFFF;
}

void RedPixels::kernel1D_CopyPixels(const uint32_t* a_data, size_t a_dataSize, PixelInfo* a_outPixels)
{
  for(uint32_t i = 0; i<a_dataSize; i++)
  {
    PixelInfo info;
    info.index     = i;
    info.value     = a_data[i];
    a_outPixels[i] = info;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename Cont, typename Pred>
Cont filter(const Cont &container, Pred predicate) 
{
  Cont result;
  std::copy_if(container.begin(), container.end(), std::back_inserter(result), predicate);
  return result;
}

void RedPixels::ProcessPixels(uint32_t* a_data, size_t a_dataSize)
{
  kernel1D_CountRedPixels(a_data, a_dataSize);
  kernel1D_FindRedPixels (a_data, a_dataSize);
  
  //kernel1D_CopyPixels(a_data, a_dataSize, m_pixelsCopy.data());
  //m_foundPixels = filter(m_pixelsCopy, [](const auto& pixel) { return PixelIsRed(pixel.value); } );

  //kernel1D_PaintRedPixelsToYellow(a_data);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void process_image_cpu(std::vector<uint32_t>& a_inPixels)
{
  RedPixels filter;

  filter.SetMaxDataSize(a_inPixels.size());
  filter.ProcessPixels(a_inPixels.data(), a_inPixels.size());
  
  //a_outPixels = filter.GetFoundPixels();
  //assert(a_outPixels.size() == filter.GetRedPixelsAmount());

  std::cout << "[cpu]: m_redPixelsNum     = " << filter.m_redPixelsNum << std::endl;
  std::cout << "[cpu]: m_otherPixelsNum   = " << filter.m_otherPixelsNum << std::endl;
  std::cout << "[cpu]: m_testPixelsAmount = " << filter.m_testPixelsAmount << std::endl;
  std::cout << "[cpu]: m_foundPixels_size = " << filter.m_foundPixels.size() << std::endl;
  std::cout << "[cpu]: m_testMin(float)   = " << filter.m_testMin << std::endl;
  std::cout << "[cpu]: m_testMax(float)   = " << filter.m_testMax << std::endl;

  return;
}