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
	EData = new float[n_outputs];

	FillEye();

	Tmp0Data = new float[n_states];
	Tmp1Data = new float[n_states*n_states]; //ss
	Tmp2Data = new float[n_outputs*n_states]; //so
	Tmp3Data = new float[n_outputs*n_outputs];//oo
	Tmp4Data = new float[n_outputs*n_outputs];//oo
	Tmp5Data = new float[n_states*n_outputs]; //so
	Tmp6Data = new float[n_states*n_outputs]; //so

	Tmp7Data = new float[n_states*n_states];//ss
	Tmp8Data = new float[n_states*n_states]; //ss
	Tmp9Data = new float[n_states*n_states]; //ss



	arm_mat_init_f32(&Jf, n_states, n_states, JfData);
	arm_mat_init_f32(&Jh, n_outputs, n_states, JhData);
//
	arm_mat_init_f32(&Qn, n_states, n_states, QnData);
	arm_mat_init_f32(&Rn, n_outputs, n_outputs, RnData);

	arm_mat_init_f32(&Kk, n_states, n_outputs, KkData);
	arm_mat_init_f32(&Pk, n_states, n_states, PkData);

	arm_mat_init_f32(&Xest, n_states, 1u, XestData); //Vetor
	arm_mat_init_f32(&U, n_inputs, 1u, UData); //Vetor
	arm_mat_init_f32(&Y, n_outputs, 1u, YData); //Vetor
	arm_mat_init_f32(&Yest, n_outputs, 1u, YestData); //Vetor
	arm_mat_init_f32(&I, n_states, n_states, IData); //Identidade
	arm_mat_init_f32(&E, n_outputs, 1u, EData); //Identidade

	arm_mat_init_f32(&KkE, n_states, 1, Tmp0Data);
	arm_mat_init_f32(&JfPkJf_, n_states, n_states, Tmp1Data);
	arm_mat_init_f32(&JhPk, n_outputs, n_states, Tmp2Data);
	arm_mat_init_f32(&JhPkJh_, n_outputs, n_outputs, Tmp3Data);
	arm_mat_init_f32(&S, n_outputs, n_outputs, Tmp4Data);
	arm_mat_init_f32(&Jh_, n_states, n_outputs, Tmp5Data);
	arm_mat_init_f32(&PkJh_, n_states, n_outputs, Tmp6Data);

	arm_mat_init_f32(&JfPk, n_states, n_states, Tmp7Data);
	arm_mat_init_f32(&Jf_, n_states, n_states, Tmp8Data);
	arm_mat_init_f32(&Pk_update, n_states, n_states, Tmp9Data);

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
	delete Tmp7Data;
	delete Tmp8Data;
	delete Tmp9Data;
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

	//PREDICT
	//x = f(x,u)
	StateFun(XestData,UData,Xest);
	Jacobian_F(XestData,UData,Jf);

	//Pk = APkA' + Qn
	arm_mat_mult_f32(&Jf, &Pk, &JfPk); // Pk = Jf*Pk
	arm_mat_trans_f32(&Jf, &Jf_); // JfTrans = Jf'
	arm_mat_mult_f32(&JfPk, &Jf_, &JfPkJf_); // Pk = A*Pk*A'
	arm_mat_add_f32(&JfPkJf_, &Qn, &Pk); // Pk = A*Pk*A' + Qn

}

void Kalman::EKF::Update(const float* Output){

	memcpy(YData,Output,n_outputs*sizeof(float));

	Jacobian_H(XestData,UData,Jh);
	MeasureFun(XestData,UData,Yest);

	//UPDATE
	//y = y - Cx
	arm_mat_sub_f32(&Y, &Yest, &E); // Yest = Y - Yest
	//S = C*Pk*C' + Rn
	arm_mat_mult_f32(&Jh, &Pk, &JhPk); //C*Pk
	arm_mat_trans_f32(&Jh, &Jh_); // C'
	arm_mat_mult_f32(&JhPk, &Jh_, &JhPkJh_); // C*Pk*C'
	arm_mat_add_f32(&JhPkJh_, &Rn, &JhPkJh_);// C*Pk*C' + Rn
	//S = inv(C*Pk*C' + Rn);
	arm_mat_inverse_f32(&JhPkJh_, &S);
	//K = Pk*C'*S
	arm_mat_mult_f32(&Pk, &Jh_, &PkJh_); // Pk*C'
	arm_mat_mult_f32(&PkJh_, &S, &Kk); // Kk = Pk*C'*S;
	//X = X + Ky
	arm_mat_mult_f32(&Kk, &E, &KkE);
	arm_mat_add_f32(&Xest, &KkE, &Xest);
	//Pk = (I- K*C)*Pk
	arm_mat_mult_f32(&Kk, &Jh, &JfPkJf_);
	arm_mat_sub_f32(&I, &JfPkJf_, &JfPkJf_);
	arm_mat_mult_f32(&JfPkJf_, &Pk, &Pk_update);
	//Copia Pk_update para Pk
	memcpy(Pk.pData,Pk_update.pData,sizeof(float)*n_states*n_states);


}




} /* namespace Kalman */
