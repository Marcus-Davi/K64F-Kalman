/*
 * EKF.h
 *
 *  Created on: 13 de fev de 2020
 *      Author: marcus
 */

#ifndef EKF_H_
#define EKF_H_

#include "arm_math.h"

//Ponteiro pra funções (jacobiana, processo, measurement, ...)
// f(x,u) -> Estados Xk, Entradas Uk
typedef void (*FunctionPt)(const float* Xk,const float *Uk,arm_matrix_instance_f32* M); // M é entrada,saída ou uma matriz

namespace Kalman {

class EKF {
public:
	EKF();
	virtual ~EKF();


private:

	unsigned int n_states;
	unsigned int n_outputs;
	unsigned int n_inputs;


	arm_matrix_instance_f32 Jf; // n x n
	arm_matrix_instance_f32 Jh; // out x out


	arm_matrix_instance_f32 Qn; // n x n
	arm_matrix_instance_f32 Rn; // out x out

	arm_matrix_instance_f32 Kk; // n x out
	arm_matrix_instance_f32 Pk; // n x n

	arm_matrix_instance_f32 X_est; // n x 1
	arm_matrix_instance_f32 Y; // out x 1
	arm_matrix_instance_f32 Y_est; // out x 1
	arm_matrix_instance_f32 U; // in x 1


	arm_matrix_instance_f32 I; // n x n

	arm_matrix_instance_f32 tmp0; //n x 1 -> A*x, B*u, K*y
	arm_matrix_instance_f32 tmp1; //n x n -> A*Pk*A', Kk*C
	arm_matrix_instance_f32 tmp2; //out x n -> C*Pk
	arm_matrix_instance_f32 tmp3; //out x out -> C*Pk*C'
	arm_matrix_instance_f32 tmp4; //out x out -> out x out |||| TESTAR com tmp3 primeiro
	arm_matrix_instance_f32 tmp5; //n x out -> C'
	arm_matrix_instance_f32 tmp6; //n x out Pk*C'
};

} /* namespace Kalman */

#endif /* EKF_H_ */
