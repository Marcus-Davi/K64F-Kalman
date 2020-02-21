/*
 * EKF_Stack.h
 *
 *  Created on: 20 de fev de 2020
 *      Author: marcus
 */

#ifndef EKF_STACK_H_
#define EKF_STACK_H_

#include "arm_math.h"
#include "KalmanFunProt.h"

// Here we define explicitly the filter dimensions
#define N_STATES 4
#define N_OUTPUTS 6
#define N_INPUTS 3


namespace Kalman {

class EKF_Stack {

public:
	EKF_Stack();
	virtual ~EKF_Stack();

	inline void SetQn(const float* qndata){
		memcpy(Qn.pData,qndata,N_STATES*N_STATES*sizeof(float)); //eficiente
	}
	inline void SetRn(const float* rndata){
		memcpy(Rn.pData,rndata,N_OUTPUTS*N_OUTPUTS*sizeof(float)); //eficiente
	}
	inline void SetX0(const float* x0){
		memcpy(Xest.pData,x0,N_STATES*sizeof(float)); //eficiente
	}


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

	inline unsigned int GetBytesUsed(){
		return bytes_used;
	}

	inline float* GetEstimatedState(){
		return Xest.pData;
	}

	void Predict(const float* Input);
	void Update(const float* Output);

private:

	void FillEye();
	void FillPk();

	unsigned int bytes_used = sizeof(float)*(N_STATES*N_STATES*7 + N_STATES*N_OUTPUTS*5 + N_OUTPUTS*N_OUTPUTS*3 +
			N_STATES*2 + N_OUTPUTS*3 + N_INPUTS);

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
	arm_matrix_instance_f32 E; // out x 1


	arm_matrix_instance_f32 I; // n x n

	arm_matrix_instance_f32 KkE; //n x 1 -> A*x, B*u, K*y
	arm_matrix_instance_f32 JfPkJf_; //n x n -> A*Pk*A', Kk*C
	arm_matrix_instance_f32 JhPk; //out x n -> C*Pk
	arm_matrix_instance_f32 JhPkJh_; //out x out -> C*Pk*C'
	arm_matrix_instance_f32 S; //out x out -> out x out |||| TESTAR com tmp3 primeiro
	arm_matrix_instance_f32 Jh_; //n x out -> C'
	arm_matrix_instance_f32 PkJh_; //n x out Pk*C'



	arm_matrix_instance_f32 JfPk; // n x n
	arm_matrix_instance_f32 Jf_; // n x out
	arm_matrix_instance_f32 Pk_update; // n x m


};

}

#endif /* EKF_STACK_H_ */
