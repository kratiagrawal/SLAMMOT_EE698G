#include <iostream>
#include <Eigen/Eigen>
using namespace Eigen;

int main(){
	int N=0, v, w, t;

	VectorXf u(3);
	MatrixXf Cov(3,3);
	MatrixXd Fx(3,3);
	MatrixXf R(3,3);
	MatrixXf Q(3,3);

	VectorXf PM(3);
	MatrixXf G(3);
	MatrixXf g(3,3);

	u << 0,0,0;
	Cov << 	0,0,0,
			0,0,0
			0,0,0;

	R 	<<	0,0,0,
			0,0,0
			0,0,0;

	Q 	<<	0,0,0,
			0,0,0
			0,0,0;

	Fx <<	1,0,0,
			0,1,0
			0,0,1;

	PM << 	-(v/w)*sin(u(3)) + (v/w)*sin(u(3) + w*t),
			 (v/w)*sin(u(3)) - (v/w)*sin(u(3) + w*t),
			 				w*t;
	g << 0, 0, (v/w)*cos(u(3)) - (v/w)*cos(u(3) + w*t),
		 0, 0, (v/w)*sin(u(3)) - (v/w)*sin(u(3) + w*t),
		 0,0,0;
			 			

while(1){

//	Fx.conservativeResize(3, u.size());

	// Prediction Step
	u = u + Fx.transpose()*PM;
	G =  MatrixXf::Identity(rows-3,cols-3) + Fx.transpose()*g*fx;
	Cov = G*Cov*G.transpose() + Fx.transpose()*R*Fx;

	for(i = 1; i<= myFrame.size(); i++){
  		u.conservativeResize(3N+3);
  		float ri = sqrt(pow(x-u(0),2) + pow(y-u(1),2));
  		float bear = atan2(y,x);
  		Vector3f V;
  		V << cos(bear + u(2)),
  			 sin(bear + u(2)),
  			 		z/ri;
  		u.tail(3) << (u.head(2),0) + ri*V;

	}

}


}