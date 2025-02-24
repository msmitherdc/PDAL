/******************************************************************************
* Copyright (c) 2023, Howard Butler (howard@hobu.co)*
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
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
*
****************************************************************************/
#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"
#include <io/LasReader.hpp>
#include <io/FauxReader.hpp>
#include "../io/ArrowWriter.hpp"

namespace pdal
{
namespace arrow
{

TEST(ArrowWriterTest, write_array_feather)
{

    Options readerOps;
    readerOps.add("filename", Support::datapath("las/1.2-with-color.las"));
    LasReader reader;
    reader.setOptions(readerOps);

    Options writerOps;
    writerOps.add("filename", Support::temppath("simple.feather"));
    writerOps.add("batch_size", 3);
    ArrowWriter writer;
    writer.setInput(reader);
    writer.setOptions(writerOps);

    PointTable table;
    writer.prepare(table);
    PointViewSet viewSet = writer.execute(table);

}

TEST(ArrowWriterTest, write_array_parquet)
{
    Options readerOps;
    readerOps.add("bounds", BOX3D(0, 200, 1000, 99, 299, 1099));
    readerOps.add("mode", "ramp");
    readerOps.add("count", 100);
    FauxReader reader;
    reader.setOptions(readerOps);

    Options writerOps;
    writerOps.add("filename", Support::temppath("simple.parquet"));
    writerOps.add("format", "parquet");
    ArrowWriter writer;
    writer.setInput(reader);
    writer.setOptions(writerOps);

    PointTable table;
    writer.prepare(table);
    PointViewSet viewSet = writer.execute(table);
}

} // namespace arrow
} // namespace pdal

