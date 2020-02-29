/* 
 * Software License Agreement (BSD 3-Clause License)
 * 
 * Copyright (c) 2020, Yoshiaki Sato
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cl_manager.h"
using namespace std;
void initializeCL(cv::ocl::Context &context)
{
    if (!cv::ocl::haveOpenCL())
    {
        cout << "OpenCL is not avaiable..." << endl
             << flush;
        throw "OpenCL is not avaiable...";
    }
    if (!context.create(cv::ocl::Device::TYPE_GPU))
    {
        cout << "Failed creating the context..." << endl;
        throw "Failed creating the context...";
    }

    // In OpenCV 3.0.0 beta, only a single device is detected.
    cout << context.ndevices() << " GPU devices are detected." << endl;
    for (size_t i = 0; i < context.ndevices(); i++)
    {
        cv::ocl::Device device = context.device(i);
        cout << "name                 : " << device.name() << endl;
        cout << "available            : " << device.available() << endl;
        cout << "imageSupport         : " << device.imageSupport() << endl;
        cout << "OpenCL_C_Version     : " << device.OpenCL_C_Version() << endl;
        cout << endl;
    }

    // Select the first device
    cv::ocl::Device(context.device(0));
}

void getKernel(const char *kernel_code_file_name, const char *kernel_function, cv::ocl::Kernel &kernel, cv::ocl::Context &context, std::string build_opt)
{
    static const char* kernel_name = nullptr;
    static std::shared_ptr<cv::ocl::ProgramSource> program_source;

    if(kernel_name != kernel_code_file_name)
    {
        std::ifstream ifs(kernel_code_file_name);
        std::string kernelSource((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        program_source = std::make_shared<cv::ocl::ProgramSource>(kernelSource);
        kernel_name = kernel_code_file_name;
    }

    if (0)
    {
        std::ifstream ifs(kernel_code_file_name);
        std::string kernelSource((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        cv::ocl::ProgramSource program_source(kernelSource);
    }

    // Compile the kernel code
    cv::String errmsg;
    cv::ocl::Program program = context.getProg(*program_source, build_opt, errmsg);
    if (NULL == program.ptr())
    {
        std::cerr << errmsg << std::endl
                  << std::flush;
        throw "getProg failed.";
    }

    kernel = cv::ocl::Kernel(kernel_function, program);
}