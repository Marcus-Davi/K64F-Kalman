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

float KQ_Xest[N_STATES] = {1.0,0,0,0}; //Quaternion body2nav
float KQ_U[N_INPUTS];
float KQ_Y[N_OUTPUTS];
float KQ_Yest[N_OUTPUTS];

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
float KQ_tmp1Pk[N_STATES*N_STATES];

namespace Kalman {

EKF_Stack::EKF_Stack(){

	arm_mat_init_f32(&Jf, N_STATES, N_STATES, KQ_Jf);
	arm_mat_init_f32(&Jh, N_OUTPUTS, N_STATES, KQ_Jh);
//
	arm_mat_init_f32(&Qn, N_STATES, N_STATES, KQ_Qn);
	arm_mat_init_f32(&Rn, N_OUTPUTS, N_OUTPUTS, KQ_Rn);

	arm_mat_init_f32(&Kk, N_STATES, N_OUTPUTS, KQ_Kk);
	arm_mat_init_f32(&Pk, N_STATES, N_STATES, KQ_Pk);


//	arm_mat_init_f32(&KalmanStruct->X, n_states, 1u, Xdata); //Vetor
	arm_mat_init_f32(&Xest, N_STATES, 1u, KQ_Xest); //Vetor
	arm_mat_init_f32(&U, N_INPUTS, 1u, KQ_U); //Vetor
	arm_mat_init_f32(&Y, N_OUTPUTS, 1u, KQ_Y); //Vetor
	arm_mat_init_f32(&Yest, N_OUTPUTS, 1u, KQ_Yest); //Vetor
	arm_mat_init_f32(&I, N_STATES, N_STATES, KQ_I); //Identidade

	arm_mat_init_f32(&tmp0, N_STATES, 1, KQ_tmp0);
	arm_mat_init_f32(&tmp1, N_STATES, N_STATES, KQ_tmp1);
	arm_mat_init_f32(&tmp2, N_OUTPUTS, N_STATES, KQ_tmp2);
	arm_mat_init_f32(&tmp3, N_OUTPUTS, N_OUTPUTS, KQ_tmp3);
	arm_mat_init_f32(&tmp4, N_OUTPUTS, N_OUTPUTS, KQ_tmp4);
	arm_mat_init_f32(&tmp5, N_STATES, N_OUTPUTS, KQ_tmp5);
	arm_mat_init_f32(&tmp6, N_STATES, N_OUTPUTS, KQ_tmp6);

	arm_mat_init_f32(&Jf_Pk, N_STATES, N_STATES, KQ_Jf_Pk);
	arm_mat_init_f32(&JfTrans, N_STATES, N_STATES, KQ_JfTrans);
	arm_mat_init_f32(&tmp1Pk, N_STATES, N_STATES, KQ_tmp1Pk);


}

void EKF_Stack::Predict(float* system_input){
	U.pData = system_input; //Entrada de Controle

	//PREDICT
	//x = f(x,u)
	StateFun(Xest.pData,U.pData,&Xest);
	Jacobian_F(Xest.pData,U.pData,&Jf);

	//Pk = APkA' + Qn
	arm_mat_mult_f32(&Jf, &Pk, &Jf_Pk); // Pk = Jf*Pk
	arm_mat_trans_f32(&Jf, &JfTrans); // JfTrans = Jf'
	arm_mat_mult_f32(&Jf_Pk, &JfTrans, &tmp1); // Pk = A*Pk*A'
	arm_mat_add_f32(&tmp1, &Qn, &Pk); // Pk = A*Pk*A' + Qn


}

void EKF_Stack::Update(float* system_output){
	Y.pData = system_output; //Leitura

	MeasureFun(Xest.pData,U.pData,&Yest);
	Jacobian_H(Xest.pData,U.pData,&Jh);


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
	arm_mat_mult_f32(&tmp1, &Pk, &tmp1Pk);
	memcpy(Pk.pData,tmp1Pk.pData,sizeof(float)*N_STATES*N_STATES);

}

}

