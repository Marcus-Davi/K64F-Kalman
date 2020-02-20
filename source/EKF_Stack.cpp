/*
 * EKF_Stack.cpp
 *
 *  Created on: 20 de fev de 2020
 *      Author: marcus
 */

#include "EKF_Stack.h"


float KQ_Jf[N_STATES*N_STATES];
float KQ_Jh[N_OUTPUTS*N_STATES];
float KQ_Kk[N_STATES*N_OUTPUTS];

float KQ_Qn[N_STATES*N_STATES]; //3x3 n x n
float KQ_Rn[N_OUTPUTS*N_OUTPUTS]; //3x3 out x out

float KQ_Pk[N_STATES*N_STATES];

float KQ_Xest[N_STATES]; //Quaternion body2nav
float KQ_U[N_INPUTS];
float KQ_Y[N_OUTPUTS];
float KQ_Yest[N_OUTPUTS];
float KQ_E[N_OUTPUTS];

float KQ_I[N_STATES*N_STATES] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

float KQ_tmp0[N_STATES]; // n x 1
float KQ_tmp1[N_STATES*N_STATES]; // n x n
float KQ_tmp2[N_OUTPUTS*N_STATES]; //out x n
float KQ_tmp3[N_OUTPUTS*N_OUTPUTS]; // out x out
float KQ_tmp4[N_OUTPUTS*N_OUTPUTS]; // out x out
float KQ_tmp5[N_STATES*N_OUTPUTS]; // n x out
float KQ_tmp6[N_STATES*N_OUTPUTS];// n x out


float KQ_Jf_Pk[N_STATES*N_STATES];
float KQ_JfTrans[N_STATES*N_STATES];
float KQ_PkUpdate[N_STATES*N_STATES];

//TODO Melhorar escrita

namespace Kalman {

EKF_Stack::EKF_Stack(){

	arm_mat_init_f32(&Jf, N_STATES, N_STATES, KQ_Jf);
	arm_mat_init_f32(&Jh, N_OUTPUTS, N_STATES, KQ_Jh);
//
	arm_mat_init_f32(&Qn, N_STATES, N_STATES, KQ_Qn);
	arm_mat_init_f32(&Rn, N_OUTPUTS, N_OUTPUTS, KQ_Rn);

	arm_mat_init_f32(&Kk, N_STATES, N_OUTPUTS, KQ_Kk);
	arm_mat_init_f32(&Pk, N_STATES, N_STATES, KQ_Pk);


	arm_mat_init_f32(&Xest, N_STATES, 1u, KQ_Xest); //Vetor
	arm_mat_init_f32(&U, N_INPUTS, 1u, KQ_U); //Vetor
	arm_mat_init_f32(&Y, N_OUTPUTS, 1u, KQ_Y); //Vetor
	arm_mat_init_f32(&Yest, N_OUTPUTS, 1u, KQ_Yest); //Vetor
	arm_mat_init_f32(&I, N_STATES, N_STATES, KQ_I); //Identidade
	arm_mat_init_f32(&E, N_OUTPUTS, 1u, KQ_E); //Identidade

	arm_mat_init_f32(&KkE, N_STATES, 1, KQ_tmp0);
	arm_mat_init_f32(&JfPkJf_, N_STATES, N_STATES, KQ_tmp1);
	arm_mat_init_f32(&JhPk, N_OUTPUTS, N_STATES, KQ_tmp2);
	arm_mat_init_f32(&JhPkJh_, N_OUTPUTS, N_OUTPUTS, KQ_tmp3);
	arm_mat_init_f32(&S, N_OUTPUTS, N_OUTPUTS, KQ_tmp4);
	arm_mat_init_f32(&Jh_, N_STATES, N_OUTPUTS, KQ_tmp5);
	arm_mat_init_f32(&PkJh_, N_STATES, N_OUTPUTS, KQ_tmp6);

	arm_mat_init_f32(&JfPk, N_STATES, N_STATES, KQ_Jf_Pk);
	arm_mat_init_f32(&Jf_, N_STATES, N_STATES, KQ_JfTrans);
	arm_mat_init_f32(&Pk_update, N_STATES, N_STATES, KQ_PkUpdate);


}

void EKF_Stack::Predict(float* system_input){
	U.pData = system_input; //Entrada de Controle

	//PREDICT
	//x = f(x,u)
	StateFun(Xest.pData,U.pData,&Xest);
	Jacobian_F(Xest.pData,U.pData,&Jf);

	//Pk = APkA' + Qn
	arm_mat_mult_f32(&Jf, &Pk, &JfPk); // Pk = Jf*Pk
	arm_mat_trans_f32(&Jf, &Jf_); // JfTrans = Jf'
	arm_mat_mult_f32(&JfPk, &Jf_, &JfPkJf_); // Pk = A*Pk*A'
	arm_mat_add_f32(&JfPkJf_, &Qn, &Pk); // Pk = A*Pk*A' + Qn


}

void EKF_Stack::Update(float* system_output){
	Y.pData = system_output; //Leitura

	MeasureFun(Xest.pData,U.pData,&Yest);
	Jacobian_H(Xest.pData,U.pData,&Jh);

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
	memcpy(Pk.pData,Pk_update.pData,sizeof(float)*N_STATES*N_STATES);

}

}

