/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2019, ITU/ISO/IEC
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
*    be used to endorse or promote products derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \file     TEncTemporalFilter.cpp
\brief    TEncTemporalFilter class
*/
#include "TEncTemporalFilter.h"
#include <math.h>


template <typename T>
void allocateMatrix(T ***matrix, Int width, Int height)
{
  (*matrix) = new T*[height];
  (*matrix)[0] = new T[width * height];
  for (Int i = 1; i < height; i++)
    (*matrix)[i] = (*matrix)[i - 1] + width;
}

template <typename T>
void deallocateMatrix(T ***matrix)
{
  delete[](*matrix)[0];
  delete[](*matrix);
}

void PaddedPelMap::create(Int pad, Int w, Int h)
{
  m_padding = pad;
  m_width = w;
  m_height = h;
  m_stride = w + 2 * pad;

  m_origin = (Pel*)xMalloc(Pel, m_stride * (h + 2 * pad));
  memset(m_origin, 0, sizeof(Pel) * m_stride * (h + 2 * pad));

  m_originCB = (Pel*)xMalloc(Pel, (m_stride * (h + 2 * pad)) >> 2);
  memset(m_originCB, 0, sizeof(Pel) * ((m_stride * (h + 2 * pad)) >> 2));

  m_originCR = (Pel*)xMalloc(Pel, (m_stride * (h + 2 * pad)) >> 2);
  memset(m_originCR, 0, sizeof(Pel) * ((m_stride * (h + 2 * pad)) >> 2));
}

void PaddedPelMap::create(Int pad, Int w, Int h, const TComPicYuv *src)
{
  create(pad, w, h);
  copyFromPelStorage(src);
}

void PaddedPelMap::destroy()
{
  xFree(m_origin);
  xFree(m_originCB);
  xFree(m_originCR);
}

void PaddedPelMap::copyFromPelStorage(const TComPicYuv *source)
{
  // fill picture area
  for (Int i = 0; i < m_height; i++)
  {
    memcpy(get(0, i), source->getAddr(COMPONENT_Y) + i * source->getStride(COMPONENT_Y), m_width * sizeof(Pel));
  }

  for (Int i = 0; i < m_height / 2; i++)
  {
    memcpy(getCB(0, i), source->getAddr(COMPONENT_Cb) + i * source->getStride(COMPONENT_Cb), (m_width / 2) * sizeof(Pel));
    memcpy(getCR(0, i), source->getAddr(COMPONENT_Cr) + i * source->getStride(COMPONENT_Cr), (m_width / 2) * sizeof(Pel));
  }

  fillPadding();
}

void PaddedPelMap::copyToPelStorage(TComPicYuv *dest)
{
  for (Int i = 0; i < m_height; i++)
  {
    memcpy(dest->getAddr(COMPONENT_Y) + i * dest->getStride(COMPONENT_Y), get(0, i), m_width * sizeof(Pel));
  }

  for (Int i = 0; i < m_height / 2; i++)
  {
    memcpy(dest->getAddr(COMPONENT_Cb) + i * dest->getStride(COMPONENT_Cb), getCB(0, i), m_width / 2 * sizeof(Pel));
    memcpy(dest->getAddr(COMPONENT_Cr) + i * dest->getStride(COMPONENT_Cr), getCR(0, i), m_width / 2 * sizeof(Pel));
  }
}

void memseti(Pel* pointer, Pel value, Int num)
{
  for (Int i = 0; i < num; i++)
  {
    pointer[i] = value;
  }
}

void PaddedPelMap::fillPadding()
{
  // fill padding top + bottom
  for (Int i = 0; i < m_padding; i++)
  {
    memseti(&m_origin[i * m_stride], at(0, 0), m_padding);
    memcpy(&m_origin[i * m_stride + m_padding], get(0, 0), m_width * sizeof(Pel));
    memset(&m_origin[i * m_stride + m_padding + m_width], at(m_width - 1, 0), m_padding); // top right
    memseti(&m_origin[(i + m_padding + m_height) * m_stride], at(0, m_height - 1), m_padding);
    memcpy(&m_origin[(i + m_padding + m_height) * m_stride + m_padding], get(0, m_height - 1), m_width * sizeof(Pel));
    memseti(&m_origin[(i + m_padding + m_height) * m_stride + m_padding + m_width], at(m_width - 1, m_height - 1), m_padding);
  }
  // fill padding left + right
  for (Int i = 0; i < m_height; i++)
  {
    memseti(&m_origin[(m_padding + i) * m_stride], at(0, i), m_padding);
    memseti(&m_origin[(m_padding + i) * m_stride + m_padding + m_width], at(m_width - 1, i), m_padding);
  }

  for (Int i = 0; i < m_padding / 2; i++)
  {
    memseti(&m_originCB[i * m_stride / 2], atCB(0, 0), m_padding / 2);
    memcpy(&m_originCB[i * m_stride / 2 + m_padding / 2], getCB(0, 0), (m_width / 2) * sizeof(Pel));
    memset(&m_originCB[i * m_stride / 2 + m_padding / 2 + m_width / 2], atCB(m_width / 2 - 1, 0), m_padding / 2); // top right
    memseti(&m_originCB[(i + m_padding / 2 + m_height / 2) * m_stride / 2], atCB(0, m_height / 2 - 1), m_padding / 2);
    memcpy(&m_originCB[(i + m_padding / 2 + m_height / 2) * m_stride / 2 + m_padding / 2], getCB(0, m_height / 2 - 1), m_width / 2 * sizeof(Pel));
    memseti(&m_originCB[(i + m_padding / 2 + m_height / 2) * m_stride / 2 + m_padding / 2 + m_width / 2], atCB(m_width / 2 - 1, m_height / 2 - 1), m_padding / 2);

    memseti(&m_originCR[i * m_stride / 2], atCR(0, 0), m_padding / 2);
    memcpy(&m_originCR[i * m_stride / 2 + m_padding / 2], getCR(0, 0), (m_width / 2) * sizeof(Pel));
    memset(&m_originCR[i * m_stride / 2 + m_padding / 2 + m_width / 2], atCR(m_width / 2 - 1, 0), m_padding / 2); // top right
    memseti(&m_originCR[(i + m_padding / 2 + m_height / 2) * m_stride / 2], atCR(0, m_height / 2 - 1), m_padding / 2);
    memcpy(&m_originCR[(i + m_padding / 2 + m_height / 2) * m_stride / 2 + m_padding / 2], getCR(0, m_height / 2 - 1), m_width / 2 * sizeof(Pel));
    memseti(&m_originCR[(i + m_padding / 2 + m_height / 2) * m_stride / 2 + m_padding / 2 + m_width / 2], atCR(m_width / 2 - 1, m_height / 2 - 1), m_padding / 2);
  }
  // fill padding left + right
  for (Int i = 0; i < m_height / 2; i++)
  {
    memseti(&m_originCB[(m_padding / 2 + i) * m_stride / 2], atCB(0, i), m_padding / 2);
    memseti(&m_originCB[(m_padding / 2 + i) * m_stride / 2 + m_padding / 2 + m_width / 2], atCB(m_width / 2 - 1, i), m_padding / 2);

    memseti(&m_originCR[(m_padding / 2 + i) * m_stride / 2], atCR(0, i), m_padding / 2);
    memseti(&m_originCR[(m_padding / 2 + i) * m_stride / 2 + m_padding / 2 + m_width / 2], atCR(m_width / 2 - 1, i), m_padding / 2);
  }
}

Pel PaddedPelMap::at(Int x, Int y) const
{
  assert(x >= -m_padding && y >= -m_padding);
  assert(x < (Int)(m_width + m_padding) && y < (Int)(m_height + m_padding));
  return m_origin[m_padding * m_stride + m_padding + y * m_stride + x];
}

Pel* PaddedPelMap::get(Int x, Int y) const
{
  assert(x >= -m_padding && y >= -m_padding);
  assert(x < (Int)(m_width + m_padding) && y < (Int)(m_height + m_padding));
  return &m_origin[m_padding * m_stride + m_padding + y * m_stride + x];
}

Pel PaddedPelMap::atCB(Int x, Int y) const
{
  assert(x >= -m_padding / 2 && y >= -m_padding / 2);
  assert(x < (Int)(m_width / 2 + m_padding / 2) && y < (Int)(m_height / 2 + m_padding / 2));
  return m_originCB[m_padding / 2 * m_stride / 2 + m_padding / 2 + y * m_stride / 2 + x];
}

Pel* PaddedPelMap::getCB(Int x, Int y) const
{
  assert(x >= -m_padding / 2 && y >= -m_padding / 2);
  assert(x < (Int)(m_width / 2 + m_padding / 2) && y < (Int)(m_height / 2 + m_padding / 2));
  return &m_originCB[m_padding / 2 * m_stride / 2 + m_padding / 2 + y * m_stride / 2 + x];
}

Pel PaddedPelMap::atCR(Int x, Int y) const
{
  assert(x >= -m_padding / 2 && y >= -m_padding / 2);
  assert(x < (Int)(m_width / 2 + m_padding / 2) && y < (Int)(m_height / 2 + m_padding / 2));
  return m_originCR[m_padding / 2 * m_stride / 2 + m_padding / 2 + y * m_stride / 2 + x];
}

Pel* PaddedPelMap::getCR(Int x, Int y) const
{
  assert(x >= -m_padding / 2 && y >= -m_padding / 2);
  assert(x < (Int)(m_width / 2 + m_padding / 2) && y < (Int)(m_height / 2 + m_padding / 2));
  return &m_originCR[m_padding / 2 * m_stride / 2 + m_padding / 2 + y * m_stride / 2 + x];
}

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

const Int TEncTemporalFilter::s_range = 2;
const Double TEncTemporalFilter::s_chromaFactor = 0.55;
const Double TEncTemporalFilter::s_sigmaMultiplier = 9.0;
const Double TEncTemporalFilter::s_sigmaZeroPoint = 10.0;
const Int TEncTemporalFilter::s_motionVectorFactor = 16;
const Int TEncTemporalFilter::s_padding = 128;
const Int TEncTemporalFilter::s_interpolationFilter[16][8] =
{
    {   0,   0,   0,  64,   0,   0,   0,   0 },   //0
    {   0,   1,  -3,  64,   4,  -2,   0,   0 },   //1 -->-->
    {   0,   1,  -6,  62,   9,  -3,   1,   0 },   //2 -->
    {   0,   2,  -8,  60,  14,  -5,   1,   0 },   //3 -->-->
    {   0,   2,  -9,  57,  19,  -7,   2,   0 },   //4
    {   0,   3, -10,  53,  24,  -8,   2,   0 },   //5 -->-->
    {   0,   3, -11,  50,  29,  -9,   2,   0 },   //6 -->
    {   0,   3, -11,  44,  35, -10,   3,   0 },   //7 -->-->
    {   0,   1,  -7,  38,  38,  -7,   1,   0 },   //8
    {   0,   3, -10,  35,  44, -11,   3,   0 },   //9 -->-->
    {   0,   2,  -9,  29,  50, -11,   3,   0 },   //10-->
    {   0,   2,  -8,  24,  53, -10,   3,   0 },   //11-->-->
    {   0,   2,  -7,  19,  57,  -9,   2,   0 },   //12
    {   0,   1,  -5,  14,  60,  -8,   2,   0 },   //13-->-->
    {   0,   1,  -3,   9,  62,  -6,   1,   0 },   //14-->
    {   0,   0,  -2,   4,  64,  -3,   1,   0 }    //15-->-->
};

const Double TEncTemporalFilter::s_refStrengths[3][2] =
{ // abs(POC offset)
  //  1,    2
  {0.85, 0.60},  // s_range * 2
  {1.20, 1.00},  // s_range
  {0.30, 0.30}   // otherwise
};

TEncTemporalFilter::TEncTemporalFilter() :
  m_FrameSkip(0),
  m_chromaFormatIDC(NUM_CHROMA_FORMAT),
  m_sourceWidth(0),
  m_sourceHeight(0),
  m_iQP(0),
  m_iGOPSize(0),
  m_framesToBeEncoded(0),
  m_bClipInputVideoToRec709Range(false),
  m_inputColourSpaceConvert(NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS),
  m_uiMaxCUWidth(0),
  m_uiMaxCUHeight(0),
  m_uiMaxTotalCUDepth(0)
{}

void TEncTemporalFilter::init(Int frameSkip, const Int inputBitDepth[MAX_NUM_CHANNEL_TYPE],
  const Int MSBExtendedBitDepth[MAX_NUM_CHANNEL_TYPE],  const Int internalBitDepth[MAX_NUM_CHANNEL_TYPE],
  Int width, Int height, Int *pad, Int frames, Bool Rec709, std::string filename, ChromaFormat chroma,
  InputColourSpaceConversion colorSpaceConv, Int maxCUWidth, Int maxCUHeight, Int maxTotalCUDepth, Int QP,
  Int iGOPSize, std::map<Int, Double> temporalFilterStrengths, Bool gopBasedTemporalFilterFutureReference)
{
  m_FrameSkip = frameSkip;
  for (Int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_inputBitDepth[i] = inputBitDepth[i];
    m_MSBExtendedBitDepth[i] = MSBExtendedBitDepth[i];
    m_internalBitDepth[i] = internalBitDepth[i];
  }

  m_sourceWidth = width;
  m_sourceHeight = height;
  for (Int i = 0; i < 2; i++)
  {
    m_aiPad[i] = pad[i];
  }
  m_framesToBeEncoded = frames;
  m_bClipInputVideoToRec709Range = Rec709;
  m_inputFileName = filename;
  m_chromaFormatIDC = chroma;
  m_inputColourSpaceConvert = colorSpaceConv;
  m_uiMaxCUHeight = maxCUHeight;
  m_uiMaxCUWidth = maxCUWidth;
  m_uiMaxTotalCUDepth = maxTotalCUDepth;
  m_iQP = QP;
  m_iGOPSize = iGOPSize;
  m_temporalFilterStrengths = temporalFilterStrengths;
  m_gopBasedTemporalFilterFutureReference = gopBasedTemporalFilterFutureReference;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Bool TEncTemporalFilter::filter(TComPicYuv *orgPic, Int receivedPoc)
{
  Bool isFilterThisFrame = false;
  if (m_iQP >= 17)  // disable filter for QP < 17
  {
    for (map<Int, Double>::iterator it = m_temporalFilterStrengths.begin(); it != m_temporalFilterStrengths.end(); ++it)
    {
      Int filteredFrame = it->first;
      if (receivedPoc % filteredFrame == 0)
      {
        isFilterThisFrame = true;
        break;
      }
    }
  }

  if (isFilterThisFrame)
  {
    Int offset = m_FrameSkip;
    TVideoIOYuv yuvFrames;
    yuvFrames.open(m_inputFileName, false, m_inputBitDepth, m_MSBExtendedBitDepth, m_internalBitDepth);
    yuvFrames.skipFrames(std::max(offset + receivedPoc - s_range, 0), m_sourceWidth - m_aiPad[0], m_sourceHeight - m_aiPad[1], m_chromaFormatIDC);

    TComPicYuv     picBuffers[s_range << 1];
    TComPicYuv     dummyPicBufferTO; // Only used temporary in yuvFrames.read 

    MotionVector **mvs[s_range << 1];
    PaddedPelMap   ppm[s_range << 1];
    Int            origOffsets[s_range << 1];

    Int firstFrame = receivedPoc + offset - s_range;
    Int lastFrame = receivedPoc + offset + s_range;
    if (!m_gopBasedTemporalFilterFutureReference)
    {
      lastFrame = receivedPoc + offset - 1;
    }
    Int refs = 0;
    Int origOffset = -s_range;

    // subsample original picture so it only needs to be done once
    PaddedPelMap ppm_orig;
    ppm_orig.create(s_padding, m_sourceWidth, m_sourceHeight, orgPic);
    subsample(ppm_orig, &m_orgSub2);
    subsample(m_orgSub2, &m_orgSub4);

    // determine motion vectors
    for (Int poc = firstFrame; poc <= lastFrame; poc++)
    {
      if (poc < 0)
      {
        origOffset++;
        continue; // frame not available
      }
      else if (poc == offset + receivedPoc)
      { // hop over frame that will be filtered
        yuvFrames.skipFrames(1, m_sourceWidth - m_aiPad[0], m_sourceHeight - m_aiPad[1], m_chromaFormatIDC);
        origOffset++;
        continue;
      }
      picBuffers[refs].create(m_sourceWidth, m_sourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxTotalCUDepth, true);
      dummyPicBufferTO.create(m_sourceWidth, m_sourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight, m_uiMaxTotalCUDepth, true);
      if (!yuvFrames.read(&picBuffers[refs], &dummyPicBufferTO, m_inputColourSpaceConvert, m_aiPad, m_chromaFormatIDC, m_bClipInputVideoToRec709Range))
      {
        return false; // eof or read fail
      }

      allocateMatrix(&mvs[refs], m_sourceWidth / 4, m_sourceHeight / 4);
      ppm[refs].create(s_padding, m_sourceWidth, m_sourceHeight, &picBuffers[refs]);
      motionEstimation(mvs[refs], ppm_orig, ppm[refs]);
      origOffsets[refs] = origOffset;
      refs++;
      origOffset++;
    }

    // filter
    PaddedPelMap newOrgPic;
    newOrgPic.create(s_padding, m_sourceWidth, m_sourceHeight);
    Double overallStrength = -1.0;
    for (map<Int, Double>::iterator it = m_temporalFilterStrengths.begin(); it != m_temporalFilterStrengths.end(); ++it)
    {
      Int frame = it->first;
      Double strength = it->second;
      if (receivedPoc % frame == 0)
      {
        overallStrength = strength;
      }
    }
    bilateralFilter(ppm_orig, mvs, ppm, refs, &newOrgPic, origOffsets, overallStrength);

    // move filtered to orgPic
    newOrgPic.copyToPelStorage(orgPic);

    // clean up
    m_orgSub2.destroy();
    m_orgSub4.destroy();
    newOrgPic.destroy();
    ppm_orig.destroy();
    dummyPicBufferTO.destroy();
    for (Int j = 0; j < refs; j++)
    {
      picBuffers[j].destroy();
      deallocateMatrix(&mvs[j]);
    }

    yuvFrames.close();
    return true;
  }
  return false;
}

// ====================================================================================================================
// Private member functions
// ====================================================================================================================

void TEncTemporalFilter::subsample(const PaddedPelMap input, PaddedPelMap *output, const Int factor)
{
  const Int newWidth = input.m_width / factor;
  const Int newHeight = input.m_height / factor;
  output->create(s_padding, newWidth, newHeight);
  for (Int y = 0; y < newHeight; y++)
  {
    Int y2 = y << 1;
    Pel *inRow = input.get(0, y2);
    Pel *inRowBelow = input.get(0, y2 + 1);
    Pel *target = output->get(0, y);
    for (Int x = 0; x < newWidth; x++)
    {
      target[x] = (inRow[0] + inRowBelow[0] + inRow[1] + inRowBelow[1] + 2) >> 2;
      inRow += 2;
      inRowBelow += 2;
    }
  }
  output->fillPadding();
}

Int TEncTemporalFilter::motionError(const PaddedPelMap orig, PaddedPelMap buffer, const Int x, const Int y, Int dx, Int dy, const Int bs, const Int besterror = 8 * 8 * 1024 * 1024)
{
  Int error = 0;// dx * 10 + dy * 10;
  if (((dx | dy) & 0xF) == 0)
  {
    dx /= s_motionVectorFactor;
    dy /= s_motionVectorFactor;
    for (Int y1 = 0; y1 < bs; y1++)
    {
      Pel* origRowStart = orig.get(x, y + y1);
      Pel* bufferRowStart = buffer.get(x + dx, y + y1 + dy);
      for (Int x1 = 0; x1 < bs; x1 += 2)
      {
        Int diff = origRowStart[x1] - bufferRowStart[x1];
        error += diff * diff;
        diff = origRowStart[x1 + 1] - bufferRowStart[x1 + 1];
        error += diff * diff;
      }
      if (error > besterror)
      {

        return error;
      }
    }
  }
  else
  {
    const Int *xFilter = s_interpolationFilter[dx & 0xF];
    const Int *yFilter = s_interpolationFilter[dy & 0xF];
    Int tempArray[64 + 8][64];

    Int iSum, iBase;
    for (Int y1 = 1; y1 < bs + 7; y1++)
    {
      const Int yOffset = y + y1 + (dy >> 4) - 3;
      const Pel *sourceRow = buffer.get(0, yOffset);
      for (Int x1 = 0; x1 < bs; x1++)
      {
        iSum = 0;
        iBase = x + x1 + (dx >> 4) - 3;
        const Pel *rowStart = sourceRow + iBase;

        iSum += xFilter[1] * rowStart[1];
        iSum += xFilter[2] * rowStart[2];
        iSum += xFilter[3] * rowStart[3];
        iSum += xFilter[4] * rowStart[4];
        iSum += xFilter[5] * rowStart[5];
        iSum += xFilter[6] * rowStart[6];

        tempArray[y1][x1] = iSum;
      }
    }
    for (Int y1 = 0; y1 < bs; y1++)
    {
      const Pel *origRow = orig.get(0, y + y1);
      for (Int x1 = 0; x1 < bs; x1++)
      {
        iSum = 0;
        iSum += yFilter[1] * tempArray[y1 + 1][x1];
        iSum += yFilter[2] * tempArray[y1 + 2][x1];
        iSum += yFilter[3] * tempArray[y1 + 3][x1];
        iSum += yFilter[4] * tempArray[y1 + 4][x1];
        iSum += yFilter[5] * tempArray[y1 + 5][x1];
        iSum += yFilter[6] * tempArray[y1 + 6][x1];

        iSum = (iSum + (1 << 11)) >> 12;
        iSum = iSum < 0 ? 0 : (iSum > 1023 ? 1023 : iSum);

        error += (iSum - origRow[x + x1]) * (iSum - origRow[x + x1]);
      }
      if (error > besterror)
        return error;
    }
  }
  return error;
}

void TEncTemporalFilter::motionEstimation(MotionVector **vectors, const PaddedPelMap orig, PaddedPelMap buffer, const Int blockSize,
  MotionVector **previous, const Int factor, const Bool doubleRes)
{
  Int range = 5;
  const Int stepSize = blockSize;

  for (Int blockY = 0; blockY + blockSize < orig.m_height; blockY += stepSize)
  {
    for (Int blockX = 0; blockX + blockSize < orig.m_width; blockX += stepSize)
    {
      MotionVector best;

      if (previous == NULL)
      {
        range = 8;
      }
      else
      {
        for (Int py = -2; py <= 2; py++)
        {
          Int testy = blockY / (2 * blockSize) + py;
          for (Int px = -2; px <= 2; px++)
          {
            Int testx = blockX / (2 * blockSize) + px;
            if ((testx >= 0) && (testx < orig.m_width / (2 * blockSize)) && (testy >= 0) && (testy < orig.m_height / (2 * blockSize)))
            {
              MotionVector old = previous[testy][testx];
              Int error = motionError(orig, buffer, blockX, blockY, old.x * factor, old.y * factor, blockSize, best.error);
              if (error < best.error)
              {
                best.set(old.x * factor, old.y * factor, error);
              }
            }
          }
        }
      }
      MotionVector prevBest = best;
      for (Int y2 = prevBest.y / s_motionVectorFactor - range; y2 <= prevBest.y / s_motionVectorFactor + range; y2++)
      {
        for (Int x2 = prevBest.x / s_motionVectorFactor - range; x2 <= prevBest.x / s_motionVectorFactor + range; x2++)
        {
          Int error = motionError(orig, buffer, blockX, blockY, x2 * s_motionVectorFactor, y2 * s_motionVectorFactor, blockSize, best.error);
          if (error < best.error)
          {
            best.set(x2 * s_motionVectorFactor, y2 * s_motionVectorFactor, error);
          }
        }
      }
      if (doubleRes)
      { // merge into one loop, probably with precision array (here [12, 3] or maybe [4, 1]) with setable number of iterations
        prevBest = best;
        Int doubleRange = 3 * 4;
        for (Int y2 = prevBest.y - doubleRange; y2 <= prevBest.y + doubleRange; y2 += 4)
        {
          for (Int x2 = prevBest.x - doubleRange; x2 <= prevBest.x + doubleRange; x2 += 4)
          {
            Int error = motionError(orig, buffer, blockX, blockY, x2, y2, blockSize, best.error);
            if (error < best.error)
            {
              best.set(x2, y2, error);
            }

          }
        }

        prevBest = best;
        doubleRange = 3;
        for (Int y2 = prevBest.y - doubleRange; y2 <= prevBest.y + doubleRange; y2++)
        {
          for (Int x2 = prevBest.x - doubleRange; x2 <= prevBest.x + doubleRange; x2++)
          {
            Int error = motionError(orig, buffer, blockX, blockY, x2, y2, blockSize, best.error);
            if (error < best.error)
            {
              best.set(x2, y2, error);
            }

          }
        }

      }
      vectors[blockY / stepSize][blockX / stepSize] = best;
    }
  }
}

void TEncTemporalFilter::motionEstimation(MotionVector **mv, PaddedPelMap orgPic, PaddedPelMap buffer)
{
  const Int width = m_sourceWidth;
  const Int height = m_sourceHeight;
  MotionVector **mv_0;
  MotionVector **mv_1;
  MotionVector **mv_2;
  allocateMatrix(&mv_0, width / 16, height / 16);
  allocateMatrix(&mv_1, width / 16, height / 16);
  allocateMatrix(&mv_2, width / 16, height / 16);

  PaddedPelMap bufferSub2;
  PaddedPelMap bufferSub4;

  subsample(buffer, &bufferSub2);
  subsample(bufferSub2, &bufferSub4);

  motionEstimation(mv_0, m_orgSub4, bufferSub4, 16);
  motionEstimation(mv_1, m_orgSub2, bufferSub2, 16, mv_0, 2);
  motionEstimation(mv_2, orgPic, buffer, 16, mv_1, 2);

  motionEstimation(mv, orgPic, buffer, 8, mv_2, 1, true);

  // clean up stuff
  bufferSub2.destroy();
  bufferSub4.destroy();
  deallocateMatrix(&mv_0);
  deallocateMatrix(&mv_1);
  deallocateMatrix(&mv_2);
}

void TEncTemporalFilter::applyMotion(MotionVector **vectors, PaddedPelMap input, PaddedPelMap *output)
{
  const Int bs = 8;
  for (Int y = 0; y + bs <= m_sourceHeight; y += bs)
  {
    for (Int x = 0; x + bs <= m_sourceWidth; x += bs)
    {
      MotionVector mv = vectors[y / bs][x / bs];
      const Int startx = 0;
      const Int starty = 0;

      Int tempArray[64 + 8][64];
      const Int *xFilter = s_interpolationFilter[mv.x & 0xF];
      const Int *yFilter = s_interpolationFilter[mv.y & 0xF];
      Int iSum, iBase;
      for (Int by = 1; by < bs + 7; by++)
      {
        const Int yOffset = y + by + (mv.y >> 4) - 3;
        const Pel *sourceRow = input.get(0, yOffset);
        for (Int bx = 0; bx < bs; bx++)
        {
          iSum = 0;
          iBase = x + bx + (mv.x >> 4) - 3;
          const Pel *rowStart = sourceRow + iBase;

          iSum += xFilter[1] * rowStart[1];
          iSum += xFilter[2] * rowStart[2];
          iSum += xFilter[3] * rowStart[3];
          iSum += xFilter[4] * rowStart[4];
          iSum += xFilter[5] * rowStart[5];
          iSum += xFilter[6] * rowStart[6];

          tempArray[by][bx] = iSum;
        }
      }
      for (Int by = starty; by < bs; by++)
      {
        for (Int bx = startx; bx < bs; bx++)
        {
          iSum = 0;

          iSum += yFilter[1] * tempArray[by + 1][bx];
          iSum += yFilter[2] * tempArray[by + 2][bx];
          iSum += yFilter[3] * tempArray[by + 3][bx];
          iSum += yFilter[4] * tempArray[by + 4][bx];
          iSum += yFilter[5] * tempArray[by + 5][bx];
          iSum += yFilter[6] * tempArray[by + 6][bx];

          iSum = (iSum + (1 << 11)) >> 12;
          iSum = iSum < 0 ? 0 : (iSum > 1023 ? 1023 : iSum);
          *output->get(x + bx, y + by) = iSum;
        }
      }
    }
  }

  for (Int y = 0; y + bs / 2 <= m_sourceHeight / 2; y += bs / 2)
  {
    for (Int x = 0; x + bs / 2 <= m_sourceWidth / 2; x += bs / 2)
    {
      MotionVector vector = vectors[(y * 2) / bs][(x * 2) / bs];
      Int dx = vector.x >> 1;
      Int dy = vector.y >> 1;
      Int starty = 0;
      Int startx = 0;

      //Q-pel
      Int yInt = vector.y >> 5;
      Int xInt = vector.x >> 5;
      Int xFrac = dx & 15;
      Int yFrac = dy & 15;

      Int tempArray[64 + 8][64];
      const Int* xFilter = s_interpolationFilter[xFrac];
      const Int* yFilter = s_interpolationFilter[yFrac];
      Int isum, ibase;
      for (Int iy = 0; iy < bs / 2 + 8; iy++)
      {
        Int yc = y + iy + yInt - 3;
        const Pel* sourceRow = input.getCB(0, yc);

        for (Int ix = 0; ix < bs / 2; ix++)
        {
          isum = 0;
          ibase = x + ix + xInt - 3;
          const Pel* rowStart = sourceRow + ibase;

          //isum += xFilter[0] * rowStart[0];
          isum += xFilter[1] * rowStart[1];
          isum += xFilter[2] * rowStart[2];
          isum += xFilter[3] * rowStart[3];
          isum += xFilter[4] * rowStart[4];
          isum += xFilter[5] * rowStart[5];
          isum += xFilter[6] * rowStart[6];
          //isum += xFilter[7] * rowStart[7];

          tempArray[iy][ix] = isum;
        }
      }
      for (Int iy = starty; iy < bs / 2; iy++)
      {
        for (Int ix = startx; ix < bs / 2; ix++)
        {
          Int sum = 0;

          //sum += yFilter[0] * tempArray[iy][ix];
          sum += yFilter[1] * tempArray[iy + 1][ix];
          sum += yFilter[2] * tempArray[iy + 2][ix];
          sum += yFilter[3] * tempArray[iy + 3][ix];
          sum += yFilter[4] * tempArray[iy + 4][ix];
          sum += yFilter[5] * tempArray[iy + 5][ix];
          sum += yFilter[6] * tempArray[iy + 6][ix];
          //sum += yFilter[7] * tempArray[iy + 7][ix];

          sum = (sum + (1 << 11)) >> 12;
          if (sum < 0)
            sum = 0;
          if (sum > 1023)
            sum = 1023;
          *output->getCB(x + ix, y + iy) = sum;
        }
      }
    }
  }
  for (Int y = 0; y + bs / 2 <= m_sourceHeight / 2; y += bs / 2)
  {
    for (Int x = 0; x + bs / 2 <= m_sourceWidth / 2; x += bs / 2)
    {
      const MotionVector vector = vectors[(y * 2) / bs][(x * 2) / bs];
      const Int dx = vector.x >> 1;
      const Int dy = vector.y >> 1;
      const Int starty = 0;
      const Int startx = 0;

      //Qpel
      const Int yInt = vector.y >> 5;
      const Int xInt = vector.x >> 5;
      const Int xFrac = dx & 15;
      const Int yFrac = dy & 15;

      Int tempArray[64 + 8][64];
      const Int* xFilter = s_interpolationFilter[xFrac];
      const Int* yFilter = s_interpolationFilter[yFrac];
      Int isum, ibase;
      for (Int iy = 1; iy < bs / 2 + 7; iy++)
      {
        Int yc = y + iy + yInt - 3;
        const Pel* sourceRow = input.getCR(0, yc);

        for (Int ix = 0; ix < bs / 2; ix++)
        {
          isum = 0;
          ibase = x + ix + xInt - 3;
          const Pel* rowStart = sourceRow + ibase;

          isum += xFilter[1] * rowStart[1];
          isum += xFilter[2] * rowStart[2];
          isum += xFilter[3] * rowStart[3];
          isum += xFilter[4] * rowStart[4];
          isum += xFilter[5] * rowStart[5];
          isum += xFilter[6] * rowStart[6];

          tempArray[iy][ix] = isum;
        }
      }
      for (Int iy = starty; iy < bs / 2; iy++)
      {
        for (Int ix = startx; ix < bs / 2; ix++)
        {
          Int sum = 0;

          sum += yFilter[1] * tempArray[iy + 1][ix];
          sum += yFilter[2] * tempArray[iy + 2][ix];
          sum += yFilter[3] * tempArray[iy + 3][ix];
          sum += yFilter[4] * tempArray[iy + 4][ix];
          sum += yFilter[5] * tempArray[iy + 5][ix];
          sum += yFilter[6] * tempArray[iy + 6][ix];

          sum = (sum + (1 << 11)) >> 12;
          if (sum < 0)
            sum = 0;
          if (sum > 1023)
            sum = 1023;
          *output->getCR(x + ix, y + iy) = sum;
        }
      }
    }
  }
}

void TEncTemporalFilter::bilateralFilter(PaddedPelMap orgPic, MotionVector ***vectors, PaddedPelMap *picBuffer, const Int refs, PaddedPelMap *newOrgPic, const Int *origOffsets, Double overallStrength)
{
  PaddedPelMap correctedPics[2 * s_range];
  for (Int i = 0; i < refs; i++)
  {
    correctedPics[i].create(0, m_sourceWidth, m_sourceHeight);
    applyMotion(vectors[i], picBuffer[i], &correctedPics[i]);
  }

  Int refStrengthRow = 2;
  if (refs == s_range << 1)
  {
    refStrengthRow = 0;
  }
  else if (refs == s_range)
  {
    refStrengthRow = 1;
  }

  const Double lumaSigmaSq = (m_iQP - s_sigmaZeroPoint) * (m_iQP - s_sigmaZeroPoint) * s_sigmaMultiplier;

  for (Int y = 0; y < m_sourceHeight; y++)
  {
    for (Int x = 0; x < m_sourceWidth; x++)
    {
      const Int orgVal = (Int) orgPic.at(x, y);
      Double temporalWeightSum = 1.0;
      Double newVal = (Double) orgVal;
      for (Int i = 0; i < refs; i++)
      {
        const Int refVal = (Int) correctedPics[i].at(x, y);
        Double diff = (Double)(refVal - orgVal);
        Double diffSq = diff * diff;
        const Int index = std::min(1, std::abs(origOffsets[i]) - 1);
        const Double weight = 0.4 * overallStrength * s_refStrengths[refStrengthRow][index] * exp(-diffSq / (2 * lumaSigmaSq));
        newVal += weight * refVal;
        temporalWeightSum += weight;
      }
      newVal /= temporalWeightSum;
      *newOrgPic->get(x, y) = (Pel)round(newVal);
    }
  }
  
  const Double chromaSigmaSq = 30 * 30;

  for (Int y = 0; y < m_sourceHeight / 2; y++)
  {
    for (Int x = 0; x < m_sourceWidth / 2; x++)
    {
      const Int orgVal = (Int) orgPic.atCB(x, y);
      Double weightSum = 1.0;
      Double newVal = (Double) orgVal;
      for (Int i = 0; i < refs; i++)
      {
        const Int refVal = correctedPics[i].atCB(x, y);
        const Int diffSq = (refVal - orgVal) * (refVal - orgVal);
        const Int index = std::min(1, std::abs(origOffsets[i]) - 1);
        const Double weight = s_chromaFactor * overallStrength * s_refStrengths[refStrengthRow][index] * exp(-diffSq / (2 * chromaSigmaSq));
        newVal += (weight * refVal);
        weightSum += weight;
      }
      newVal /= weightSum;
      *newOrgPic->getCB(x, y) = (Pel)round(newVal);
    }
  }
  
  for (Int y = 0; y < m_sourceHeight / 2; y++)
  {
    for (Int x = 0; x < m_sourceWidth / 2; x++)
    {
      const Int orgVal = (Int) orgPic.atCR(x, y);
      Double weightSum = 1.0;
      Double newval = (Double) orgVal;
      for (Int i = 0; i < refs; i++)
      {
        const Int refVal = correctedPics[i].atCR(x, y);
        const Int diffSq = (refVal - orgVal) * (refVal - orgVal);
        const Int index = std::min(1, std::abs(origOffsets[i]) - 1);
        const Double weight = s_chromaFactor * overallStrength * s_refStrengths[refStrengthRow][index] * exp(-diffSq / (2 * chromaSigmaSq));
        newval += (weight * refVal);
        weightSum += weight;
      }
      newval /= weightSum;
      *newOrgPic->getCR(x, y) = (Pel)round(newval);
    }
  }

  for (Int i = 0; i < refs; i++)
  {
    correctedPics[i].destroy();
  }
}

//! \}
