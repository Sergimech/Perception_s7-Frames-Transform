//Sergi Baiges MASTER ROBOTICS
//EXERCICI de transformació i rotació de frames amb C++
//Per executar s'ha de fer un cmake amb el CMakeLists corresponent, un make dins, i per executar el codi ./example

//std
#include <iostream>
//eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>

int main()
{
	Eigen::Matrix3d T_bs;
	Eigen::Matrix3d T_0b;
	Eigen::Matrix3d T_0s;
	Eigen::Matrix3d rot_bs;
	Eigen::Matrix3d rot_0b;
	Eigen::Vector3d p0;
	Eigen::Vector3d mb;
	Eigen::Vector3d qs;
	Eigen::Vector3d qb;
	Eigen::Vector3d q0;
	Eigen::Vector3d q0s;

	//Declaración d'angles de rotació
	double theta_deg = 30;
	double beta_deg = 25;
	
	//Transformació de graus a radians

	double theta_rad = M_PI * theta_deg / 180 ;
	double beta_rad = M_PI * beta_deg / 180 ;

	//Declaració de vectors

	p0 << 23,30,1;
	mb << 1.5,0.75,1;
	qs << 12,5,1;

	//Declaració de les matrius de rotació

	rot_bs << cos(theta_rad),-sin(theta_rad),0,sin(theta_rad),cos(theta_rad),0,0,0,1;
	rot_0b << cos(beta_rad),-sin(beta_rad),0,sin(beta_rad),cos(beta_rad),0,0,0,1;

	//Declaració de les matrius de transformació (traslació + rotació)

	T_bs << rot_0b(0,0),rot_0b(0,1),mb(0),rot_0b(1,0),rot_0b(1,1),mb(1),0,0,1;
	T_0b << rot_bs(0,0),rot_bs(0,1),p0(0),rot_bs(1,0),rot_bs(1,1),p0(1),0,0,1;
	T_0s << rot_0b(0,0)+rot_bs(0,0),rot_0b(0,1)+rot_bs(0,1),mb(0)+p0(0),rot_0b(1,0)+rot_bs(1,0),rot_0b(1,1)+rot_bs(1,1),mb(1)+p0(1),0,0,1;

	//Volem saber q0 i  qb:
	
	qb=T_bs*qs;		//Distancia de frame-cotxe fins al punt q
	q0=T_0b*qb;		//Distancia de frame-world fins al punt q

	//Hem de trobar el sensor expressat en l'origen i comprovar que el q0 que surti ha de ser el mateix.

	T_0s = T_0b*T_bs;  //Multipliquem les transformacions amb l'ordre adequat
	q0s=T_0s*qs;

	//Print dels vectors resultants

	std::cout << "Vector del sensor al punto - qb: " <<std::endl << qb << std::endl;
	std::cout << "Vector del origen al punto - q0: " <<std::endl << q0 << std::endl;
	std::cout << "Vector del origen al punto (comprovación) - q0s: " <<std::endl << q0s << std::endl;

	return 1;  //Retornem una enter perquè ho necessita per tancar el codi

}