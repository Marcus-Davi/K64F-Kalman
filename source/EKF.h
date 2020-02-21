/*
 * EKF.h
 *
 *  Created on: 13 de fev de 2020
 *      Author: marcus
 */

#ifndef EKF_H_
#define EKF_H_

#include "KalmanFunProt.h"
#include "arm_math.h" //usar DSP


namespace Kalman {

class EKF {

	//Methods
public:
	EKF(uint8_t n_states, uint8_t n_inputs, uint8_t n_outputs);
	virtual ~EKF();

	inline void SetQn(const float* qndata){
		memcpy(Qn.pData,qndata,n_states*n_states*sizeof(float)); //eficiente
	}
	inline void SetRn(const float* rndata){
		memcpy(Rn.pData,rndata,n_outputs*n_outputs*sizeof(float)); //eficiente
	}
	inline void SetX0(const float* x0){
		memcpy(Xest.pData,x0,n_states*sizeof(float)); //eficiente
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

	unsigned int n_states;
	unsigned int n_outputs;
	unsigned int n_inputs;
	unsigned int bytes_used;

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
	float* IData;
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

	arm_matrix_instance_f32 Tmp0; // n x n
	arm_matrix_instance_f32 Tmp1; // n x n
	arm_matrix_instance_f32 Tmp2; // n x n
	arm_matrix_instance_f32 Tmp3; // out x n
	arm_matrix_instance_f32 Tmp4; // n x out
	arm_matrix_instance_f32 Tmp5; // out x out
	arm_matrix_instance_f32 Tmp6; // out x out
	arm_matrix_instance_f32 Tmp7; // n x out
	arm_matrix_instance_f32 Tmp8; // n x 1

};

} /* namespace Kalman */


#endif /* EKF_H_ */
