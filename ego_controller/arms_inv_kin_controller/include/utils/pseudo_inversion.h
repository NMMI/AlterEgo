/***
 *  Software License Agreement: BSD 3-Clause License
 *
 * Copyright (c) 2021, NMMI
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
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

// ----------------------------------------------------------------------------

/**
 * \file      pseudo_inversion.h
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
 **/
#ifndef PSEUDO_INVERSION_H
#define PSEUDO_INVERSION_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
using namespace Eigen;

inline void pseudo_inverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_,bool damped = true)
{	
	double lambda_ = damped?0.2:0.0;

	JacobiSVD<MatrixXd> svd(M_, ComputeFullU | ComputeFullV);
	JacobiSVD<MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
	MatrixXd S_ = M_;	// copying the dimensions of M_, its content is not needed.
	S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

    M_pinv_ = MatrixXd(svd.matrixV()*S_.transpose()*svd.matrixU().transpose());
}

#endif
