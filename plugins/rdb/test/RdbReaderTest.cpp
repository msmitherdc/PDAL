/******************************************************************************
* Copyright (c) 2017, Peter J. Gadomski (pete.gadomski@gmail.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, view, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include <limits>

#include <pdal/pdal_test_main.hpp>

#include <pdal/Options.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PointView.hpp>

#include "RdbReader.hpp"
#include "Config.hpp"

using namespace pdal;

Options defaultRdbReaderOptions()
{
    Options options;
    Option filename("filename", testDataPath() + "130501_232206_cut.rdb");
    options.add(filename);
    return options;
}

template <typename T>
void checkDimensionClose(const PointViewPtr view,
                         std::size_t index,
                         Dimension::Id dim,
                         T edbected)
{
    T actual = view->getFieldAs<T>(dim, index);
    EXPECT_FLOAT_EQ(edbected, actual);
}

template <typename T>
void checkDimensionEqual(const PointViewPtr view,
                         std::size_t index,
                         Dimension::Id dim,
                         T edbected)
{
    T actual = view->getFieldAs<T>(dim, index);
    EXPECT_EQ(edbected, actual);
}

void checkPoint(const PointViewPtr view, std::size_t index,
                float x, float y, float z,
                double time, double echoRange, float amplitude,
                float reflectance, float deviation,
                bool isPpsLocked, uint8_t returnNumber,
                uint8_t numberOfReturns
                )
{
    using namespace Dimension;
    checkDimensionClose(view, index, Id::X, x);
    checkDimensionClose(view, index, Id::Y, y);
    checkDimensionClose(view, index, Id::Z, z);
    checkDimensionClose(view,
                        index,
                        getTimeDimensionId(isPpsLocked),
                        time);
    checkDimensionClose(view, index, Id::EchoRange, echoRange);
    checkDimensionClose(view, index, Id::Amplitude, amplitude);
    checkDimensionClose(view, index, Id::Reflectance, reflectance);
    checkDimensionClose(view, index, Id::Deviation, deviation);
    checkDimensionEqual(view, index, Id::IsPpsLocked, isPpsLocked);
    checkDimensionEqual(view, index, Id::ReturnNumber, returnNumber);
    checkDimensionEqual(view, index, Id::NumberOfReturns, numberOfReturns);
}

TEST(RdbReaderTest, testConstructor)
{
    Options options = defaultRdbReaderOptions();
    RdbReader reader;
    reader.setOptions(options);
    EXPECT_EQ(reader.getName(), "readers.rdb");
}
