#include "robot.h"
#include <QDebug>
#define PI 3.1415

robot::robot(DH_table _dh_table)
{
    nrOfLinks=6;
    setDH(_dh_table);
    //*****************************************
    Matrix<double,6,1>   vector_zero_theta;
    vector_zero_theta << 0,0,0,0,0,0;
    theta_home=vector_zero_theta;
    //*****************************************
    goHome();
}

robot::robot()
{
    nrOfLinks=6;
    DH_table IRB120_dh;

    IRB120_dh.alpha   << 0  , -90 , 0  , -90 , 90, -90 ;
    IRB120_dh.a       << 0  ,  0  , 270,  70 , 0 ,  0  ;
    IRB120_dh.d       << 290,  0  , 0  ,  302, 0 ,  130 ;
    IRB120_dh.theta_0 << 0  , -90 , 0  ,  0  , 0 ,  180;

    setDH(IRB120_dh);
    //*****************************************
    Matrix<double,6,1>   vector_zero_theta;
    vector_zero_theta << 0,0,0,0,0,0;
    theta_home=vector_zero_theta;
    //*****************************************

    goHome();
}

int robot::setDH(DH_table _dh_table)
{
    dh_table=_dh_table;
    for(unsigned int i=0;i<nrOfLinks;++i)
    {
        dh_table.alpha(i)= _dh_table.alpha(i)*M_PI/180;
        dh_table.theta_0(i)= _dh_table.theta_0(i)*M_PI/180;
    }
    goHome();
    return 0;
}

int robot::setJoints(Matrix<double,6,1>   _theta)
{
    theta=_theta;
    for(unsigned int i=0;i<nrOfLinks;++i)
    {
        theta(i)= theta(i)*M_PI/180;
    }
    //std::cout << "Joint values are changed." <<std::endl;
    return solveFK();
}

int robot::goHome()
{
    theta=theta_home;
    int result = robot::solveFK();
    //std::cout << "The robot is at home pose now." <<std::endl;
    return result;
}

int robot::solveFK()
{
    for(unsigned int i=1;i<=nrOfLinks;++i)
    {
        Matrix3d R=Matrix3d::Identity();

        Matrix4d H_alpha=Matrix4d::Identity();
        R=AngleAxisd(dh_table.alpha(i-1),Vector3d::UnitX());
        H_alpha.block(0,0,3,3)=R;

        Matrix4d H_a=Matrix4d::Identity();
        H_a(0,3)=dh_table.a(i-1);

        Matrix4d H_theta=Matrix4d::Identity();
        R=AngleAxisd(dh_table.theta_0(i-1)+theta(i-1),Vector3d::UnitZ());
        H_theta.block(0,0,3,3)=R;

        Matrix4d H_d=Matrix4d::Identity();
        H_d(2,3)=dh_table.d(i-1);

        Links[i].frameB.H= Links[i-1].frameB.H * (H_alpha*H_a*H_theta*H_d);
    }

    //std::cout << "Forward kinematics solved." <<std::endl;
    return 0;
}

Matrix4d robot::FK(Matrix<double,6,1>   _theta)
{
    setJoints(_theta);
    return Links[nrOfLinks].frameB.H;
}

Matrix<double,3,1> robot::get_xyz(Matrix<double,6,1>   _theta)
{
    setJoints(_theta);
    Matrix4d H=Links[nrOfLinks].frameB.H;
    return H.block(0,3,3,1);
}

Matrix<double,4,1> robot::get_q(Matrix<double,6,1>   _theta)
{
    setJoints(_theta);
    Matrix3d R=Links[nrOfLinks].frameB.H.block(0,0,3,3);
    Quaterniond q(R);
    Matrix<double,4,1> v;
    v << q.w(), q.x(), q.y(), q.z();
    return v;
}

int robot::IK(Matrix4d H, Matrix<double,6,1>& IK_theta, double conf1, double conf2, double conf5)
{
    double q1, q2, q3, q4, q5, q6, q11, q12, q21, q22, q51, q52;
    double px, py, pz, Xp, Yp, Zp, Rp, R4;
    double ax, ay, az;
    double a1, a2 , a3, a4, d1, d4, d6;
    double g11, g12, g21, g22, g41, g42, g5;
    double r13, r21, r22, r23, r33;
    double s5;

    a1=dh_table.a(0);
    a2=dh_table.a(1);
    a3=dh_table.a(2);
    a4=dh_table.a(3);

    d1=dh_table.d(0);
    d4=dh_table.d(3);
    d6=dh_table.d(5);

    ax=H(0,2); ay=H(1,2); az=H(2,2);
    px=H(0,3); py=H(1,3); pz=H(2,3);

    Xp = -d6*ax + px;
    Yp = -d6*ay + py;
    Zp = -d6*az + pz - d1;
    Rp  = sqrt(pow(Xp,2)  + pow(Yp,2)  + pow(Zp,2));

    q11=atan2(Yp, Xp);
    q12=atan2(-Yp, -Xp);

    if (conf1==0)
        q1=q11;
    else
        q1=q12;

    R4=  sqrt(pow(a4,2)+pow(d4,2));
    g41= asin( Zp/Rp );
    g42= PI-asin( Zp/Rp );

    g11=acos( (pow(a3,2) + pow(Rp,2) - pow(R4,2))/(2*a3*Rp) );
    g12=-acos( (pow(a3,2) + pow(Rp,2) - pow(R4,2))/(2*a3*Rp) );

    g21=acos( (pow(a3,2) + pow(R4,2) - pow(Rp,2))/(2*a3*R4) );
    g22=-acos( (pow(a3,2) + pow(R4,2) - pow(Rp,2))/(2*a3*R4) );

    if (conf1==0)
        q21=PI/2-(g11+g41);
    else
        q22=PI/2-(g12+g42);

    g5=atan2(d4,a4);

    if (conf2==0)
    {
        q2=q21;
        q3=PI-(g5+g21);
    }
    else
    {
        q2=q22;
        q3=PI-(g5+g22);
    }


    //*************************************************************
    Matrix3d R01;
    R01= AngleAxisd(dh_table.alpha(0),Vector3d::UnitX())
        *AngleAxisd(dh_table.theta_0(0)+q1,Vector3d::UnitZ());

    Matrix3d R12;
    R12= AngleAxisd(dh_table.alpha(1),Vector3d::UnitX())
        *AngleAxisd(dh_table.theta_0(1)+q2,Vector3d::UnitZ());

    Matrix3d R23;
    R23= AngleAxisd(dh_table.alpha(2),Vector3d::UnitX())
        *AngleAxisd(dh_table.theta_0(2)+q3,Vector3d::UnitZ());

    Matrix3d R03=R01*R12*R23;
    Matrix3d R06=H.block(0,0,3,3);
    Matrix3d R36=R03.inverse()*R06;
    //*************************************************************

    r13=R36(0,2);
    r21=R36(1,0);r22=R36(1,1);r23=R36(1,2);
    r33=R36(2,2);

    if (1<pow(r23,2))
    {
        qDebug() << "no q5 solution";
        return 0;
    }

    q51=atan2(sqrt(1-pow(r23,2)),r23);
    q52=atan2(-sqrt(1-pow(r23,2)),r23);
    if (q51==0||q52==0)
    {
        qDebug() << "q5==0";
        return 0;
    }

    if (conf5==0)
        q5=q51;
    else
        q5=q52;


    s5=sin(q5);
    q4=atan2(r33/s5,-r13/s5);
    q6=atan2(r22/s5,-r21/s5);

    IK_theta << q1, q2, q3, q4, q5, q6;
    IK_theta=180/PI*IK_theta;

    return 1;
}
