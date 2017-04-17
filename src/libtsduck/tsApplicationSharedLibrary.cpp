//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2005-2017, Thierry Lelegard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
//----------------------------------------------------------------------------
//
//  Application shared libraries
//
//----------------------------------------------------------------------------

#include "tsApplicationSharedLibrary.h"
#include "tsSysUtils.h"



//----------------------------------------------------------------------------
// Constructor.
//----------------------------------------------------------------------------

ts::ApplicationSharedLibrary::ApplicationSharedLibrary (const std::string& filename, const std::string& prefix, bool permanent) :
    SharedLibrary (filename, permanent),
    _prefix (prefix)
{
    const std::string basename (BaseName (filename));
    const std::string suffix (PathSuffix (filename));
    const std::string execdir (DirectoryName (ExecutableFile()));
    const bool nodir = basename == filename;

    // If not loaded, try with standard extension
    if (!isLoaded() && suffix.empty()) {
        load (filename + SharedLibrary::Extension);
    }

    // Then, try in same directory as executable
    if (!isLoaded() && nodir) {
        load (AddPathSuffix (execdir + PathSeparator + basename, SharedLibrary::Extension));
    }

    // Finally, try in same directory as executable with prefix
    if (!isLoaded() && nodir) {
        load (AddPathSuffix (execdir + PathSeparator + prefix + basename, SharedLibrary::Extension));
    }
}


//----------------------------------------------------------------------------
// The module name is derived from the file name
//----------------------------------------------------------------------------

std::string ts::ApplicationSharedLibrary::moduleName() const
{
    const std::string name (PathPrefix (BaseName (fileName())));
    return !_prefix.empty() && name.find (_prefix) == 0 ? name.substr (_prefix.size()) : name;
}