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

#include "least_squares_method.h"

#include <vector>

namespace virtualgimbal
{

Eigen::VectorXd least_squares_method(Eigen::VectorXd &x, Eigen::VectorXd &y, int degree)
{
    int N = degree + 1;

    assert(x.size() == y.size());
    
    Eigen::MatrixXd A(N, N);

    std::vector<double> sum;
    for (int order = 0,e=2 * degree + 1; order < e; ++order)
    {
        sum.push_back(x.array().pow((double)order).sum());
    }

    for (int col = 0; col < N; ++col)
    {
        for (int row = 0; row < N; ++row)
        {
            A(row, col) = sum[row + col];
        }
    }
    Eigen::VectorXd B(N);
    for (int i = 0; i < N; ++i)
    {
        B(i) = (x.array().pow((double)i) * y.array()).sum();
    }

    return A.inverse() * B;
}

} // namespace virtualgimbal