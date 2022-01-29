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

#pragma OPENCL EXTENSION cl_khr_fp64: enable
__constant sampler_t samplerLN = CLK_NORMALIZED_COORDS_FALSE | CLK_ADDRESS_CLAMP_TO_EDGE | CLK_FILTER_NEAREST;

float triangle_area(float2 p1, float2 p2, float2 p3){
   return 0.5*fabs((p1.x-p3.x)*(p2.y-p3.y)-(p2.x-p3.x)*(p1.y-p3.y));
}

float3 baryventrid_coordinate(float2 pt, float2 p1, float2 p2, float2 p3)
{
   float total = triangle_area(p1,p2,p3);
   float a1 = triangle_area(pt,p2,p3);
   float a2 = triangle_area(p1,pt,p3);
   float a3 = triangle_area(p1,p2,pt);
   return (float3)(a1,a2,a3)/total;
}

// WARNING (FCA) Not sure what this method name should be (original name 'sign' collide with some other method in cl_kernel.h)
float triangle_sign (float2 p1, float2 p2, float2 p3)
{
    return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}

bool point_in_triangle (float2 pt, float2 v1, float2 v2, float2 v3)
{
    bool b1, b2, b3;
 
    b1 = triangle_sign(pt, v1, v2) < 0.0f;
    b2 = triangle_sign(pt, v2, v3) < 0.0f;
    b3 = triangle_sign(pt, v3, v1) < 0.0f;
      
    return ((b1 == b2) && (b2 == b3));
}

float2 warp_undistort(
   float2 p,                              // UV coordinate position in a image.
   __constant float* rotation_matrix,     // Rotation Matrix in each rows.
   float k1, float k2,float p1, float p2, // Distortion parameters.
   float2 f, float2 c,
   float2 f_dst, float2 c_dst
){
   float2 x1 = (p-c)/f;
   float r2 = dot(x1,x1);
   float2 x2 = x1*(1.f + k1*r2+k2*r2*r2);
   x2 += (float2)(2.f*p1*x1.x*x1.y+p2*(r2+2.f*x1.x*x1.x), p1*(r2+2.f*x1.y*x1.y)+2.f*p2*x1.x*x1.y);
   float3 x3 = (float3)(x2.x,x2.y,1.0);
   __constant float* R = rotation_matrix + convert_int( 9*p.y);
   float3 XYZ = (float3)(R[0] * x3.x + R[1] * x3.y + R[2] * x3.z,
                         R[3] * x3.x + R[4] * x3.y + R[5] * x3.z,
                         R[6] * x3.x + R[7] * x3.y + R[8] * x3.z);
   x2 = XYZ.xy / XYZ.z;
  return x2 * f_dst + c_dst;
}

__kernel void stabilizer_function(
   // __read_only image2d_t input,
   __global uchar4 *input,
   int input_step, int input_offset, int input_rows, int input_cols, 
   __global uchar4 *output,
   int output_step, int output_offset, int output_rows, int output_cols, 
   __constant float* rotation_matrix,        // Rotation Matrix in each rows.
   int src_step, int src_offset,
   float k1, float k2,float p1, float p2,    // Distortion parameters.
   float fx, float fy, float cx, float cy,
   float fx_dst, float fy_dst, float cx_dst, float cy_dst
)
{
   // int2 size_src = get_image_dim(input);
   int2 size_src = (int2)(input_cols,input_rows);
   int2 size_dst = (int2)(output_cols,output_rows);

   int2 uvi = (int2)(get_global_id(0),get_global_id(1));
   float2 uv = convert_float2(convert_int2(uvi));

   if(uvi.y >= (size_src.y-1))
   {
      return;
   }
   if(uvi.x >= (size_src.x-1))
   {
      return;
   }

   float2 f = (float2)(fx,fy);
   float2 c = (float2)(cx,cy);

   float2 f_dst = (float2)(fx_dst,fy_dst);
   float2 c_dst = (float2)(cx_dst,cy_dst);

   float2 uv0_ = uv;
   float2 uv1_ = uv + (float2)(1,0);
   float2 uv2_ = uv + (float2)(1,1);
   float2 uv3_ = uv + (float2)(0,1);

   float2 uv0 = warp_undistort(uv0_, rotation_matrix, k1, k2, p1, p2, f, c, f_dst, c_dst);
   float2 uv1 = warp_undistort(uv1_, rotation_matrix, k1, k2, p1, p2, f, c, f_dst, c_dst);
   float2 uv2 = warp_undistort(uv2_, rotation_matrix, k1, k2, p1, p2, f, c, f_dst, c_dst);
   float2 uv3 = warp_undistort(uv3_, rotation_matrix, k1, k2, p1, p2, f, c, f_dst, c_dst);

   
   int2 uvMin = convert_int2(floor(min(min(uv0,uv1),min(uv2,uv3))));
   int2 uvMax = convert_int2(ceil(max(max(uv0,uv1),max(uv2,uv3))));

   for(int v= uvMin.y;v<=uvMax.y;++v){
      for(int u=uvMin.x;u<=uvMax.x;++u){
         int2 uvt = (int2)(u,v);
         if(any(uvt >= size_dst)) continue;
         if(any(uvt < 0)) continue;

         float2 uw_cam;
         if(point_in_triangle(convert_float2(uvt),uv0,uv1,uv3)){
            float3 ratio = baryventrid_coordinate(convert_float2(uvt),uv0,uv1,uv3);
            uw_cam = uv0_*ratio.x+uv1_*ratio.y+uv3_*ratio.z;
         }else if(point_in_triangle(convert_float2(uvt),uv1,uv2,uv3)){
            float3 ratio = baryventrid_coordinate(convert_float2(uvt),uv1,uv2,uv3);
            uw_cam = uv1_*ratio.x+uv2_*ratio.y+uv3_*ratio.z;
         }else{
            continue;
         }
         // uint4 pixel = read_imageui(input, samplerLN, uw_cam);
         int input_index = mad24(convert_int(uw_cam.y), input_cols, convert_int(uw_cam.x));
         uchar4 pixel = (uchar4)(0,0,0,0);
         
         if(all(uw_cam >= 0.f) && all(uw_cam <= convert_float2(size_src-(int2)(1,1))))
         {
            uchar4 pixel_aa = *(__global uchar4 *)(input + input_index);
            uchar4 pixel_ba = *(__global uchar4 *)(input + input_index + 1);
            uchar4 pixel_ab = *(__global uchar4 *)(input + input_index + input_cols);
            uchar4 pixel_bb = *(__global uchar4 *)(input + input_index + input_cols + 1);

            // Bilinear interpolation
            float x_a = uw_cam.x - floor(uw_cam.x);
            float x_b = 1.f- x_a;

            float y_a = uw_cam.y - floor(uw_cam.y);
            float y_b = 1.f- y_a;

            pixel = convert_uchar4(x_b * y_b * convert_float4(pixel_aa) 
                                 + x_b * y_a * convert_float4(pixel_ab)
                                 + x_a * y_b * convert_float4(pixel_ba)
                                 + x_a * y_a * convert_float4(pixel_bb));

         }
         
         int output_index = mad24(uvt.y, output_cols, uvt.x);
         __global uchar4 *p_out = (__global uchar4 *)(output + output_index);
         *p_out = convert_uchar4(pixel);
      }
   }
}

