/*
 * EKF.cpp
 *
 *  Created on: 13 de fev de 2020
 *      Author: marcus
 */

#include <EKF.h>



namespace Kalman {

EKF::EKF(uint8_t n_states, uint8_t n_inputs, uint8_t n_outputs) {

	this->n_states = n_states;
	this->n_inputs = n_inputs;
	this->n_outputs = n_outputs;
	// TODO Auto-generated constructor stub

	JfData = new float[n_states*n_states]; //ss
	JhData = new float[n_outputs*n_states]; //so
	QnData = new float[n_states*n_states]; //ss
	RnData = new float[n_outputs*n_outputs]; //oo
	KkData = new float[n_states*n_outputs]; //so
	PkData = new float[n_states*n_states]; //ss

	FillPk();

	XestData = new float[n_states];
	YestData = new float[n_outputs];
	UData = new float[n_inputs];
	YData = new float[n_outputs];
	IData = new float[n_states*n_states]; //ss

	FillEye();

	Tmp0Data = new float[n_states];
	Tmp1Data = new float[n_states*n_states]; //ss
	Tmp2Data = new float[n_outputs*n_states]; //so
	Tmp3Data = new float[n_outputs*n_outputs];//oo
	Tmp4Data = new float[n_outputs*n_outputs];//oo
	Tmp5Data = new float[n_states*n_outputs]; //so
	Tmp6Data = new float[n_states*n_outputs]; //so



	arm_mat_init_f32(&Jf, n_states, n_states, JfData);
	arm_mat_init_f32(&Jh, n_outputs, n_states, JhData);
//
	arm_mat_init_f32(&Qn, n_states, n_states, QnData);
	arm_mat_init_f32(&Rn, n_outputs, n_outputs, RnData);

	arm_mat_init_f32(&Kk, n_states, n_outputs, KkData);
	arm_mat_init_f32(&Pk, n_states, n_states, PkData);


//	arm_mat_init_f32(&KalmanStruct->X, n_states, 1u, Xdata); //Vetor
	arm_mat_init_f32(&Xest, n_states, 1u, XestData); //Vetor
	arm_mat_init_f32(&U, n_inputs, 1u, UData); //Vetor
	arm_mat_init_f32(&Y, n_outputs, 1u, YData); //Vetor
	arm_mat_init_f32(&Yest, n_outputs, 1u, YestData); //Vetor
	arm_mat_init_f32(&I, n_states, n_states, IData); //Identidade

	arm_mat_init_f32(&tmp0, n_states, 1, Tmp0Data);
	arm_mat_init_f32(&tmp1, n_states, n_states, Tmp1Data);
	arm_mat_init_f32(&tmp2, n_outputs, n_states, Tmp2Data);
	arm_mat_init_f32(&tmp3, n_outputs, n_outputs, Tmp3Data);
	arm_mat_init_f32(&tmp4, n_outputs, n_outputs, Tmp4Data);
	arm_mat_init_f32(&tmp5, n_states, n_outputs, Tmp5Data);
	arm_mat_init_f32(&tmp6, n_states, n_outputs, Tmp6Data);

	floats_used = n_states*n_states*5 + n_states*n_outputs*5 + n_outputs*n_outputs*3 + n_states + n_outputs + n_inputs;



}

EKF::~EKF() {
	// TODO Auto-generated destructor stub

	delete JfData;
	delete JhData;
	delete QnData;
	delete RnData;
	delete KkData;
	delete PkData;

	delete XestData;
	delete YestData;
	delete UData;
	delete YData;
	delete IData;

	delete Tmp0Data;
	delete Tmp1Data;
	delete Tmp2Data;
	delete Tmp3Data;
	delete Tmp4Data;
	delete Tmp5Data;
	delete Tmp6Data;
}

void Kalman::EKF::FillEye(){

	for(unsigned int i=0;i<n_states*n_states;++i){
		if(i%(n_states+1) == 0)
		IData[i] = 1;
		else
		IData[i] = 0;

	}
}

void Kalman::EKF::FillPk(){

	for(unsigned int i=0;i<n_states*n_states;++i){
		if(i%(n_states+1) == 0)
		PkData[i] = 1;
		else
		PkData[i] = 0;

	}
}

void Kalman::EKF::Predict(const float* Input){
	//Kalman->U.pData = system_input; //Entrada de Controle

	//PREDICT
	//x = f(x,u)
	memcpy(UData,Input,n_inputs*sizeof(float));

	Jacobian_F(XestData,UData,Jf);
	StateFun(XestData,UData,Xest);

	arm_mat_mult_f32(&Jf, &Pk, &Pk); // Pk = A*Pk
	arm_mat_trans_f32(&Jf, &tmp1); // tmp1 = A'
	arm_mat_mult_f32(&Pk, &tmp1, &Pk); // Pk = A*Pk*A'
	arm_mat_add_f32(&Pk, &Qn, &Pk); // Pk = A*Pk*A' + Qn

}

void Kalman::EKF::Update(const float* Output){

	memcpy(YData,Output,n_outputs*sizeof(float));

	Jacobian_H(XestData,UData,Jh);
	MeasureFun(XestData,UData,Yest);

	//UPDATE
	//y = y - Cx
	arm_mat_sub_f32(&Y, &Yest, &Yest); // Yest = Y - Yest
	//S = C*Pk*C' + Rn
	arm_mat_mult_f32(&Jh, &Pk, &tmp2); //C*Pk
	arm_mat_trans_f32(&Jh, &tmp5); // C'
	arm_mat_mult_f32(&tmp2, &tmp5, &tmp3); // C*Pk*C'
	arm_mat_add_f32(&tmp3, &Rn, &tmp3);// C*Pk*C' + Rn
	//S = inv(S);
	arm_mat_inverse_f32(&tmp3, &tmp4);
	//K = Pk*C'*S
	arm_mat_mult_f32(&Pk, &tmp5, &tmp6); // Pk*C'
	arm_mat_mult_f32(&tmp6, &tmp4, &Kk); // Kk = Pk*C'*inv(S);
	//X = X + Ky
	arm_mat_mult_f32(&Kk, &Yest, &tmp0);
	arm_mat_add_f32(&Xest, &tmp0, &Xest);
	//Pk = (I- K*C)*Pk
	arm_mat_mult_f32(&Kk, &Jh, &tmp1);
	arm_mat_sub_f32(&I, &tmp1, &tmp1);
	arm_mat_mult_f32(&tmp1, &Pk, &Pk);
}




} /* namespace Kalman */
