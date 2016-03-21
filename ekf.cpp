#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using namespace Eigen;
using namespace std;

void update(VectorXf* u, MatrixXf* E)
{
	float v, w,t;
	v = 0.1;
	w = 0.5;
	t = 0.05;
	int N;
	N = u->size();
	MatrixXf Fx(3,N);
	Matrix3f R(3,3);
	Vector3f PM(3);
	MatrixXf G(N,N);
	Matrix3f g(3,3);
	VectorXf u_(N);
	MatrixXf E_(N,N);

	u_ = *u;
	E_= *E;

	R << Matrix3f::Identity(3,3);

	Fx << MatrixXf::Identity(3,3), MatrixXf::Zero(3,N-3);

	PM << 	-(v/w)*sin(u_(2)) + (v/w)*sin(u_(2) + w*t),
			 (v/w)*sin(u_(2)) - (v/w)*sin(u_(2) + w*t),
			 				w*t;
	g << 0, 0, (v/w)*cos(u_(2)) - (v/w)*cos(u_(2) + w*t),
		 0, 0, (v/w)*sin(u_(2)) - (v/w)*sin(u_(2) + w*t),
		 0,0,0;

	// Prediction Step
	*u = *u + Fx.transpose()*PM;
	G =  MatrixXf::Identity(N,N) + Fx.transpose()*g*Fx;
	*E = G*(*E)*G.transpose() + Fx.transpose()*R*Fx;

} 

void unmatched (VectorXf* u, MatrixXf* E, VectorXf* u_new, MatrixXf* E_new, float xk, float yk, float zk) {
	float r,phi;
	int N,l;
	N = u->size();
	l = u_new->size();
	Vector3f z_hat;
	Vector3f z;
	Matrix3f Q(3,3);
	VectorXf u_(N);
	MatrixXf E_(N,N);
	VectorXf u_new_(l);
	MatrixXf E_new_(l,l);
	MatrixXf E_ky(3,l);
	MatrixXf E_kk(3,3);
	Matrix3f Pt;
	Matrix3f Pz(3,3);
	u_ = *u;
	E_= *E;
	u_new_ = *u_new;
	E_new_ = *E_new;
	Q << Matrix3f::Identity(3,3);
	r = (xk-u_(0))*(xk-u_(0)) + (yk-u_(1))*(yk-u_(1));
	phi = atan2(yk-u_(1), xk-u_(0)) - u_(2);
	Pt << 1, 0, -r*sin(phi + u_(2)),
	      0, 1, r*cos(phi + u_(2)),
	      0, 0, 0;

	Pz << cos(phi + u_(2)), -r*sin(phi + u_(2)), 0,
	      sin(phi + u_(2)), r*cos(phi + u_(2)), 0,
	      0,0,1;
	

	E_kk = Pt*E_.block<3,3>(0,0)*Pt.transpose() + Pz*Q*Pz.transpose();
	E_ky = Pt*E_new_.block(0,0,3,l);

	u_new->conservativeResize(l+3);
	*u_new << u_new_, 
	   	  u_(0) + r*cos(phi + u_(2)),
		  u_(1) + r*sin(phi + u_(2)),
		  zk; 

	E_new->conservativeResize(l+3,l+3);
	*E_new << E_new_, E_ky.transpose(),
	          E_ky, E_kk;


}

void matched (VectorXf* u, MatrixXf* E, int k, float xk, float yk, float zk) {
	Vector2f d;
	float q;
	int N;
	N = u->size();
	Vector3f z_hat;
	Vector3f z;
	MatrixXf Fx(6,N);
	MatrixXf P(3,6);
	MatrixXf H(3,N);
	Matrix3f psi(3,3); 
	MatrixXf K(N,3);
	Matrix3f Q(3,3);
	VectorXf u_(N);
	MatrixXf E_(N,N);

	u_ = *u;
	E_= *E;
	Q << Matrix3f::Identity(3,3);
	z << xk, yk, zk;

	d(0) = u_(3*k) - u_(0);
	d(1) = u_(3*k+1) - u_(1);
	q = d.transpose()*d;
	z_hat(0) = sqrt(q);
	z_hat(1) = atan2(d(1),d(0)) - u_(2);
	z_hat(2) = u_(3*k+2);
	Fx << MatrixXf::Identity(3,3), MatrixXf::Zero(3,N-3),
	      MatrixXf::Zero(3,3*k), MatrixXf::Identity(3,3), MatrixXf::Zero(3,N-3-3*k);
	P << d(0)/sqrt(q), -d(1)/sqrt(q), 0, -d(0)/sqrt(q), d(1)/sqrt(q), 0,
	     d(1)/(q), d(0)/(q), -1/(q), -d(1)/(q), -d(0)/q, 0,
	     0, 0, 0, 0, 0, 1/q;

	H = P*Fx;
	psi = H*(*E)*H.transpose() + Q;
	K = (*E)*H.transpose()*psi.inverse();
	*u = *u + K*(z-z_hat);
	*E = (MatrixXf::Identity(N,N) - K*H)*(*E);
}

int main()
{
	MatrixXf x(4,5);
	x << 1,-1,3,0,1,
	     2,1,1.1,.99,2.02,
	     3,-1,2,4,2,
             4,-1,5,1,1;	
	
	VectorXf u(6);
	u << 0,
	0,
	0,
	1,
	1,
	2;
	MatrixXf E(6,6);
	E << 1,0,0,2,1,0,
	     0,1,0,1,2,1,
	     0,0,1,0,1,0,
	     2,1,0,2,1,0,
	     1,2,1,1,2,1,
	     0,1,0,0,1,0;

	VectorXf u_new(6);
	MatrixXf E_new(6,6);
	int N;
	update(&u,&E);
	u_new << u;
	E_new << E;
	for (int i = 0; i < x.rows(); i++) {
		if(x(i,1)==-1)
			unmatched(&u,&E,&u_new,&E_new,x(i,2),x(i,3),x(i,4));
		else
			matched(&u,&E,x(i,1),x(i,2),x(i,3),x(i,4));
	}
	N = u.size();
	u_new.head(N) = u.head(N);
	E_new.block(0,0,N,N) = E.block(0,0,N,N);
	cout << u_new << endl;
	cout << E_new << endl;
}
