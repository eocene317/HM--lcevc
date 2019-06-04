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

/** \file     TEncTemporalFilter.h
    \brief    TEncTemporalFilter class (header)
 */

#ifndef __TEMPORAL_FILTER__
#define __TEMPORAL_FILTER__
#include "TLibCommon/TComPicYuv.h"
#include "TLibVideoIO/TVideoIOYuv.h"
#include <sstream>
#include <map>

 //! \ingroup EncoderLib
 //! \{

struct MotionVector {
  Int x, y;
  Int error;
  MotionVector() : x(0), y(0), error(INT_LEAST32_MAX) {}
  void set(Int nx, Int ny, Int ne) { x = nx; y = ny; error = ne; }
};

class PaddedPelMap {
public:
  Int m_padding;
  Int m_width;
  Int m_height;
  Int m_stride;

  void create(Int padding, Int width, Int height);
  void create(Int padding, Int width, Int height, const TComPicYuv *source);
  void destroy();
  void copyFromPelStorage(const TComPicYuv *source);
  void copyToPelStorage(TComPicYuv *destination);
  void fillPadding();
  Pel  at(Int x, Int y) const;
  Pel* get(Int x, Int y) const;
  Pel  atCB(Int x, Int y) const;
  Pel* getCB(Int x, Int y) const;
  Pel  atCR(Int x, Int y) const;
  Pel* getCR(Int x, Int y) const;

private:
  Pel *m_origin;
  Pel *m_originCB;
  Pel *m_originCR;
};

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class TEncTemporalFilter
{
public:
   TEncTemporalFilter();
  ~TEncTemporalFilter() {}

  void init(Int frameSkip, const Int inputBitDepth[MAX_NUM_CHANNEL_TYPE],
    const Int MSBExtendedBitDepth[MAX_NUM_CHANNEL_TYPE], const Int InternalBitDepth[MAX_NUM_CHANNEL_TYPE],
    Int width, Int height,
    Int *pad, Int frames, Bool Rec709, std::string filename, ChromaFormat inputChroma,
    InputColourSpaceConversion colorSpaceConv, Int MaxCUWidth, Int MaxCUHeight, Int MaxTotalCUDepth, Int qp,
    Int iGOPSize, std::map<Int, Double> temporalFilterStrengths, Bool gopBasedTemporalFilterFutureReference);

  Bool filter(TComPicYuv *orgPic, Int frame);

private:
  // Private static member variables
  static const Int s_range;
  static const Double s_chromaFactor;
  static const Double s_sigmaMultiplier;
  static const Double s_sigmaZeroPoint;
  static const Int s_motionVectorFactor;
  static const Int s_padding;
  static const Int s_interpolationFilter[16][8];
  static const Double s_refStrengths[3][2];

  // Private member variables
  Int m_FrameSkip;
  std::string m_inputFileName;
  Int m_inputBitDepth[3];
  Int m_MSBExtendedBitDepth[3];
  Int m_internalBitDepth[3];
  ChromaFormat m_chromaFormatIDC;
  Int m_sourceWidth;
  Int m_sourceHeight;
  Int m_iQP;
  Int m_iGOPSize;
  std::map<Int, Double> m_temporalFilterStrengths;
  Int m_aiPad[2];
  Int m_framesToBeEncoded;
  Bool m_bClipInputVideoToRec709Range;
  InputColourSpaceConversion m_inputColourSpaceConvert;
  Bool m_gopBasedTemporalFilterFutureReference;

  Int m_uiMaxCUWidth;
  Int m_uiMaxCUHeight;
  Int m_uiMaxTotalCUDepth;

  PaddedPelMap m_orgSub2;
  PaddedPelMap m_orgSub4;

  // Private functions
  void subsample(const PaddedPelMap input, PaddedPelMap *output, const Int factor = 2);
  Int motionError(const PaddedPelMap orig, PaddedPelMap buffer, const Int x, const Int y, Int dx, Int dy, const Int bs, const Int besterror);
  void motionEstimation(MotionVector **vectors, const PaddedPelMap orig, PaddedPelMap buffer, const Int bs,
    MotionVector **previous = 0, const Int factor = 1, const Bool doubleRes = false);
  void motionEstimation(MotionVector **mv, PaddedPelMap orgPic, PaddedPelMap buffer);

  void bilateralFilter(PaddedPelMap orgPic, MotionVector ***vectors, PaddedPelMap *picBuffer, const Int refs, PaddedPelMap *newOrgPic, const Int *origOffsets, Double strength);
  void applyMotion(MotionVector **vectors, PaddedPelMap input, PaddedPelMap *output);
}; // END CLASS DEFINITION TEncTemporalFilter

//! \}

#endif // __TEMPORAL_FILTER__
