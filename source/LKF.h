/*
 * LKF.h
 *
 *  Created on: 21 de fev de 2020
 *      Author: marcus
 */

#ifndef LKF_H_
#define LKF_H_

#include "arm_math.h"

namespace Kalman {

class LKF {
public:
	LKF(uint8_t n_states, uint8_t n_inputs, uint8_t n_outputs);
	virtual ~LKF();

	inline void SetQn(const float* qndata){
	memcpy(Qn.pData,qndata,n_states*n_states*sizeof(float)); //eficiente
	}
	inline void SetRn(const float* rndata){
	memcpy(Rn.pData,rndata,n_outputs*n_outputs*sizeof(float)); //eficiente
	}
	inline void SetX0(const float* x0){
	memcpy(Xest.pData,x0,n_states*sizeof(float)); //eficiente
	}

	inline void SetA(const float* A_){
	memcpy(A.pData,A_,n_states*n_states*sizeof(float)); //eficiente
	}

	inline void SetB(const float* B_){
	memcpy(B.pData,B_,n_states*n_inputs*sizeof(float)); //eficiente
	}

	inline void SetC(const float* C_){
	memcpy(C.pData,C_,n_outputs*n_states*sizeof(float)); //eficiente
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

	//Melhorar aqui a delcaração de variáveis. tá meio lixo. criar classe "kalman matrix?"

	float* AData;
	float* BData;
	float* CData;
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
	float* Tmp9Data;


	arm_matrix_instance_f32 A; // n x n
	arm_matrix_instance_f32 B; // n x in
	arm_matrix_instance_f32 C; // out x n


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
	arm_matrix_instance_f32 Tmp9; // n x 1

};

} /* namespace Kalman */

#endif /* LKF_H_ */
