/*
 * EKF_Stack.h
 *
 *  Created on: 20 de fev de 2020
 *      Author: marcus
 */

#ifndef EKF_STACK_H_
#define EKF_STACK_H_

#include "arm_math.h"

#define N_STATES 4
#define N_OUTPUTS 6 // acc + mag
#define N_INPUTS 3 //gyroscope

typedef void (*KalmanFunctionPt)(const float* Xk,const float *Uk,arm_matrix_instance_f32* M); // M é entrada,saída ou uma matriz

namespace Kalman {

class EKF_Stack {

public:

	EKF_Stack();

	inline void SetQn(const float* qndata){
		memcpy(Qn.pData,qndata,N_STATES*N_STATES*sizeof(float)); //eficiente
	}
	inline void SetRn(const float* rndata){
		memcpy(Rn.pData,rndata,N_OUTPUTS*N_OUTPUTS*sizeof(float)); //eficiente
	}
	inline void SetX0(const float* x0){
		memcpy(Xest.pData,x0,N_STATES*sizeof(float)); //eficiente

	}

	inline float* GetEstimatedState(){
		return Xest.pData;
	}
private:

	KalmanFunctionPt StateFun;
	KalmanFunctionPt MeasureFun;
	KalmanFunctionPt Jacobian_F;
	KalmanFunctionPt Jacobian_H;

	arm_matrix_instance_f32 Jf; // n x n
	arm_matrix_instance_f32 Jh; // out x n


	arm_matrix_instance_f32 Qn; // n x n
	arm_matrix_instance_f32 Rn; // out x out

	arm_matrix_instance_f32 Kk; // n x out
	arm_matrix_instance_f32 Pk; // n x n

	arm_matrix_instance_f32 Xest; // n x 1
	arm_matrix_instance_f32 Y; // out x 1
	arm_matrix_instance_f32 Yest; // out x 1
	arm_matrix_instance_f32 U; // in x 1


	arm_matrix_instance_f32 I; // n x n

	arm_matrix_instance_f32 tmp0; //n x 1 -> A*x, B*u, K*y
	arm_matrix_instance_f32 tmp1; //n x n -> A*Pk*A', Kk*C
	arm_matrix_instance_f32 tmp2; //out x n -> C*Pk
	arm_matrix_instance_f32 tmp3; //out x out -> C*Pk*C'
	arm_matrix_instance_f32 tmp4; //out x out -> out x out |||| TESTAR com tmp3 primeiro
	arm_matrix_instance_f32 tmp5; //n x out -> C'
	arm_matrix_instance_f32 tmp6; //n x out Pk*C'



	arm_matrix_instance_f32 Jf_Pk; // n x n
	arm_matrix_instance_f32 JfTrans; // n x out
	arm_matrix_instance_f32 tmp1Pk; // n x m


public:

	inline void SetStateFunction(KalmanFunctionPt F) {
		StateFun = F;
	}
	inline void  SetStateJacobian(KalmanFunctionPt F){
		Jacobian_F = F;
	}
	inline void  SetMeasurementJacobian(KalmanFunctionPt F){
		Jacobian_H = F;
	}
	inline void SetMeasurementFunction(KalmanFunctionPt F){
		MeasureFun = F;
	}

	void Predict(float* system_input);
	void Update(float* system_output);

};

}

#endif /* EKF_STACK_H_ */
