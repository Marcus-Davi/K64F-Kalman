/*
 * EKF.h
 *
 *  Created on: 13 de fev de 2020
 *      Author: marcus
 */

#ifndef EKF_H_
#define EKF_H_

//tentar fazer memcpy mais legível!

#include "arm_math.h"
//#include "stdint.h"

//Ponteiro pra funções (jacobiana, processo, measurement, ...)
// f(x,u) -> Estados Xk, Entradas Uk
// TODO Melhorar prototipo ?

typedef void (*KalmanFunctionPt)(const float* Xk,const float *Uk,arm_matrix_instance_f32& M); // M é entrada,saída ou uma matriz

namespace Kalman {

class EKF {

	//Methods
public:
	EKF(uint8_t n_states, uint8_t n_inputs, uint8_t n_outputs);
	virtual ~EKF();

	inline void SetQn(const float* qndata){
		memcpy(QnData,qndata,n_states*n_states*sizeof(float)); //eficiente
	}
	inline void SetRn(const float* rndata){
		memcpy(RnData,rndata,n_outputs*n_outputs*sizeof(float)); //eficiente
	}
	inline void SetX0(const float* x0){
		memcpy(XestData,x0,n_states*sizeof(float)); //eficiente

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

	inline unsigned int GetFloatsUsed(){
		return floats_used;
	}

//	inline void SetInput(float* inputs){
//		memcpy(UData,inputs,n_inputs*sizeof(float));
//	}
//	inline void SetMeasurements(float* outputs){
//		memcpy(YData,outputs,n_outputs*sizeof(float));
//	}

	inline float* GetEstimatedState(){
		return XestData;
	}

	void Predict(float* Input);
	void Update(float* Output);

private:
	void FillEye();
	void FillPk();

	//Attributes
private:

	unsigned int n_states;
	unsigned int n_outputs;
	unsigned int n_inputs;
	unsigned int floats_used;

	float* JfData;
	float* JhData;
	float* QnData;
	float* RnData;

	float* KkData;
	float* PkData;

	float* XestData;
	float* YestData;
	float* YData;
	float* UData;
	float* IData; //Identity
	float* EData;

	float* Tmp0Data;
	float* Tmp1Data;
	float* Tmp2Data;
	float* Tmp3Data;
	float* Tmp4Data;
	float* Tmp5Data;
	float* Tmp6Data;
	float* Tmp7Data;
	float* Tmp8Data;
	float* Tmp9Data;


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


	arm_matrix_instance_f32 Tmp7; // n x n
	arm_matrix_instance_f32 Tmp8; // n x out
	arm_matrix_instance_f32 Tmp1; //n x n

	arm_matrix_instance_f32 Tmp0; //n x 1
	arm_matrix_instance_f32 Tmp2; //out x n
	arm_matrix_instance_f32 Tmp3; //out x out
	arm_matrix_instance_f32 Tmp4; //out x out
	arm_matrix_instance_f32 Tmp5; //n x out
	arm_matrix_instance_f32 Tmp6; //n x out
	arm_matrix_instance_f32 Tmp9; // n x m
};

} /* namespace Kalman */


#endif /* EKF_H_ */
