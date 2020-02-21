/*
 * LKF.cpp
 *
 *  Created on: 21 de fev de 2020
 *      Author: marcus
 */

#include <LKF.h>

namespace Kalman {

LKF::LKF(uint8_t n_states, uint8_t n_inputs, uint8_t n_outputs) {

	this->n_states = n_states;
	this->n_inputs = n_inputs;
	this->n_outputs = n_outputs;

		// Dynamic Allocation
		AData = new float[n_states*n_states]; //ss
		BData = new float[n_states*n_inputs]; //so
		CData = new float[n_outputs*n_states]; //so
		QnData = new float[n_states*n_states]; //ss
		RnData = new float[n_outputs*n_outputs]; //oo
		KkData = new float[n_states*n_outputs]; //so
		PkData = new float[n_states*n_states]; //ss



		XestData = new float[n_states];
		YestData = new float[n_outputs];
		UData = new float[n_inputs];
		YData = new float[n_outputs];
		IData = new float[n_states*n_states]; //ss
		EData = new float[n_outputs];



		Tmp0Data = new float[n_states*n_states]; //ss
		Tmp1Data = new float[n_states*n_states]; //ss
		Tmp2Data = new float[n_states*n_states]; //ss
		Tmp3Data = new float[n_outputs*n_states]; //so
		Tmp4Data = new float[n_states*n_outputs]; //so
		Tmp5Data = new float[n_outputs*n_outputs]; //oo
		Tmp6Data = new float[n_outputs*n_outputs]; //oo
		Tmp7Data = new float[n_states*n_outputs]; //so
		Tmp8Data = new float[n_states]; //s
		Tmp9Data = new float[n_states]; //s

		FillPk();
		FillEye();


		arm_mat_init_f32(&A, n_states, n_states, AData);
		arm_mat_init_f32(&B, n_outputs, n_inputs, BData);
		arm_mat_init_f32(&C, n_outputs, n_states, CData);

		arm_mat_init_f32(&Qn, n_states, n_states, QnData);
		arm_mat_init_f32(&Rn, n_outputs, n_outputs, RnData);

		arm_mat_init_f32(&Kk, n_states, n_outputs, KkData);
		arm_mat_init_f32(&Pk, n_states, n_states, PkData);

		arm_mat_init_f32(&Xest, n_states, 1u, XestData);
		arm_mat_init_f32(&U, n_inputs, 1u, UData);
		arm_mat_init_f32(&Y, n_outputs, 1u, YData);
		arm_mat_init_f32(&Yest, n_outputs, 1u, YestData);
		arm_mat_init_f32(&I, n_states, n_states, IData);
		arm_mat_init_f32(&E, n_outputs, 1u, EData);

		arm_mat_init_f32(&Tmp0, n_states, n_states, Tmp0Data);
		arm_mat_init_f32(&Tmp1, n_states, n_states, Tmp1Data);
		arm_mat_init_f32(&Tmp2, n_states, n_states, Tmp2Data);
		arm_mat_init_f32(&Tmp3, n_outputs, n_states, Tmp3Data);
		arm_mat_init_f32(&Tmp4, n_states, n_outputs, Tmp4Data);
		arm_mat_init_f32(&Tmp5, n_outputs, n_outputs, Tmp5Data);
		arm_mat_init_f32(&Tmp6, n_outputs, n_outputs, Tmp6Data);
		arm_mat_init_f32(&Tmp7, n_states, n_outputs, Tmp7Data);
		arm_mat_init_f32(&Tmp8, n_states, 1u, Tmp8Data);
		arm_mat_init_f32(&Tmp9, n_states, 1u, Tmp9Data);

}

LKF::~LKF() {
	// TODO Auto-generated destructor stub
	delete AData;
	delete BData;
	delete CData;
	delete QnData;
	delete RnData;

	delete KkData;
	delete PkData;

	delete XestData;
	delete YestData;
	delete YData;
	delete UData;
	delete IData;
	delete EData;

	delete Tmp0Data;
	delete Tmp1Data;
	delete Tmp2Data;
	delete Tmp3Data;
	delete Tmp4Data;
	delete Tmp5Data;
	delete Tmp6Data;
	delete Tmp7Data;
	delete Tmp8Data;
}


void LKF::FillEye(){

	for(unsigned int i=0;i<n_states*n_states;++i){
		if(i%(n_states+1) == 0)
		IData[i] = 1;
		else
		IData[i] = 0;

	}
}

void LKF::FillPk(){

	for(unsigned int i=0;i<n_states*n_states;++i){
		if(i%(n_states+1) == 0)
		PkData[i] = 1;
		else
		PkData[i] = 0;

	}
}

void LKF::Predict(const float* Input){
	memcpy(UData,Input,n_inputs*sizeof(float));

	//PREDICT
	//x = Ax + Bu
	arm_mat_mult_f32(&A, &Xest, &Tmp8);
	arm_mat_mult_f32(&B, &U, &Tmp9);
	arm_mat_add_f32(&Tmp8, &Tmp9, &Xest);


	//Pk = JfPkJf' + Qn
	arm_mat_mult_f32(&A, &Pk, &Tmp0); // Tpm0 = A*Pk
	arm_mat_trans_f32(&A, &Tmp1); // Tmp1 = A'
	arm_mat_mult_f32(&Tmp0, &Tmp1, &Tmp2); // Tmp2 = A*Pk*A'
	arm_mat_add_f32(&Tmp2, &Qn, &Pk); // Pk = A*Pk*A' + Qn
}

void LKF::Update(const float* Output){
	memcpy(YData,Output,n_outputs*sizeof(float));

	//UPDATE

	// Y = Cx
	arm_mat_mult_f32(&C, &Xest, &Yest);

	//e = y - yest
	arm_mat_sub_f32(&Y, &Yest, &E); // Yest = Y - Yest
	//S = C*Pk*C' + Rn
	arm_mat_mult_f32(&C, &Pk, &Tmp3); //Tmp3 = C*Pk
	arm_mat_trans_f32(&C, &Tmp4); // Tmp4 = C'
	arm_mat_mult_f32(&Tmp3, &Tmp4, &Tmp5); // Tmp5 = C*Pk*C'
	arm_mat_add_f32(&Tmp5, &Rn, &Tmp5);// Tmp5 = C*Pk*C' + Rn

	arm_mat_inverse_f32(&Tmp5, &Tmp6); //Tmp6 = inv(C*Pk*C' + Rn);
	//K = Pk*Jh'*inv(S)
	arm_mat_mult_f32(&Pk, &Tmp4, &Tmp7); // Pk*Jh'
	arm_mat_mult_f32(&Tmp7, &Tmp6, &Kk); // Kk = Pk*Jh'/S;
	//X = X + K*e
	arm_mat_mult_f32(&Kk, &E, &Tmp8);
	arm_mat_add_f32(&Xest, &Tmp8, &Xest);
	//Pk = (I- K*C)*Pk
	arm_mat_mult_f32(&Kk, &C, &Tmp0); // Tmp0 = Kk*Jh
	arm_mat_sub_f32(&I, &Tmp0, &Tmp0); // Tmp0 = (I - Kk*Jh)
	arm_mat_mult_f32(&Tmp0, &Pk, &Tmp1); // Tmp1 = (I - kk*Jh)*Pk

	memcpy(Pk.pData,Tmp1.pData,sizeof(float)*n_states*n_states);
}

} /* namespace Kalman */
