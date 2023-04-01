#include "Mahony.h"

Mahony::Mahony()
{
    initialise();
}

Mahony::Mahony(float kp, float ki, float Ts)
{
    initialise();
    Setup(kp, ki, Ts);
}

void Mahony::Update(Eigen::Vector3f& gyro, Eigen::Vector3f& acc)
{
    Eigen::Vector3f g_n(                                          2.0f * ( m_quat.x()*m_quat.z() - m_quat.w()*m_quat.y() ),
                                                                  2.0f * ( m_quat.y()*m_quat.z() + m_quat.w()*m_quat.x() ),
                         ( m_quat.w()*m_quat.w() - m_quat.x()*m_quat.x() - m_quat.y()*m_quat.y() + m_quat.z()*m_quat.z() )  );
    Eigen::Vector3f e = acc.normalized().cross( g_n.normalized() );

    updateOrientation(gyro, e);
}

void Mahony::Update(Eigen::Vector3f& gyro, Eigen::Vector3f& acc, Eigen::Vector3f& mag)
{
    Eigen::Matrix3f R = m_quat.toRotationMatrix();

    Eigen::Vector3f g_n = R.block<1,3>(2,0).transpose();
    Eigen::Vector3f e = acc.normalized().cross( g_n.normalized() );

    Eigen::Vector3f h = R * mag.normalized();
    h(2) = 0.0f;
    Eigen::Vector3f b(h.norm(), 0.0f, 0.0f);
    e += R.transpose() * h.cross(b);

    updateOrientation(gyro, e);
}

Eigen::Quaternionf Mahony::GetOrientationAsQuaternion()
{
    return m_quat;
}

Eigen::Vector3f Mahony::GetOrientationAsRPYAngles()
{
    return m_rpy;
}

Mahony::~Mahony()
{
    
}

void Mahony::Setup(float& kp, float& ki, float& Ts)
{
    m_kp = kp;
    m_ki = ki;
    m_Ts = Ts;
}

void Mahony::initialise()
{
    m_quat.setIdentity();
    m_bias.setZero();
    m_rpy.setZero();
}

Eigen::Vector3f Mahony::quat2rpy(Eigen::Quaternionf& quat)
{
	// ||quat|| = 1
	Eigen::Vector3f rpy;
	// roll
	rpy(0) = atan2f( quat.y() * quat.z() + quat.w() * quat.x(), 0.5f - ( quat.x() * quat.x() + quat.y() * quat.y() ) );
	// pitch
	float sinarg = -2.0f * ( quat.x() * quat.z() - quat.w() * quat.y() );
	if (sinarg > 1.0f)
		sinarg = 1.0f;
	if (sinarg < -1.0f)
		sinarg = -1.0f;
	rpy(1) = asinf( sinarg );
	// yaw
	rpy(2) = atan2f( quat.x() * quat.y() + quat.w() * quat.z(), 0.5f - ( quat.y() * quat.y() + quat.z() * quat.z() ) );

	return rpy;
}

void Mahony::updateOrientation(Eigen::Vector3f& gyro, Eigen::Vector3f& e)
{
    m_bias += m_ki * e * m_Ts;
    Eigen::Matrix<float, 4, 3> Q;
    Q << -m_quat.x(), -m_quat.y(), -m_quat.z(),
          m_quat.w(), -m_quat.z(),  m_quat.y(),
          m_quat.z(),  m_quat.w(), -m_quat.x(),
         -m_quat.y(),  m_quat.x(),  m_quat.w();

    // carefull here, Eigen Quaternions have the internal storage order [x y z w] but you inilialise them with quat(w, x, y, z)
    // so I rather type the following explizitly
    Eigen::Vector4f dquat = m_Ts * 0.5f * Q * ( gyro + m_bias + m_kp * e );
    m_quat.w() += dquat(0);
    m_quat.x() += dquat(1);
    m_quat.y() += dquat(2);
    m_quat.z() += dquat(3);
    m_quat.normalize();

    m_rpy = quat2rpy(m_quat);
}