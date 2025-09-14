#include "ConfigurationRepresentation.h"

ConfigurationRepresentation::ConfigurationRepresentation(
		Configuration *angle_regulator, Configuration *dist_regulator,
		Configuration *angle_acc, Configuration *dist_acc,
		Configuration *speed_right, Configuration *speed_left)
{
	m_angle_regulator = angle_regulator;
	m_dist_regulator = dist_regulator;
	m_angle_acc = angle_acc;
	m_dist_acc = dist_acc;
	m_speed_right = speed_right;
	m_speed_left = speed_left;
}

void ConfigurationRepresentation::generateRepresentation(QCBOREncodeContext &EncodeCtx)
{
	QCBOREncode_OpenMap(&EncodeCtx);
	QCBOREncode_OpenMapInMapSZ(&EncodeCtx, "dist_regulator");
	m_dist_regulator->getConfiguration( EncodeCtx);
	QCBOREncode_CloseMap(&EncodeCtx);

	QCBOREncode_OpenMapInMapSZ(&EncodeCtx, "angle_regulator");
	m_angle_regulator->getConfiguration( EncodeCtx);
	QCBOREncode_CloseMap(&EncodeCtx);

	QCBOREncode_OpenMapInMapSZ(&EncodeCtx, "dist_acc");
	m_dist_acc->getConfiguration( EncodeCtx);
	QCBOREncode_CloseMap(&EncodeCtx);
	
	QCBOREncode_OpenMapInMapSZ(&EncodeCtx, "angle_acc");
	m_angle_acc->getConfiguration( EncodeCtx );
	QCBOREncode_CloseMap(&EncodeCtx);
	

	QCBOREncode_OpenMapInMapSZ(&EncodeCtx, "speed_right");
	m_speed_right->getConfiguration( EncodeCtx);
	QCBOREncode_CloseMap(&EncodeCtx);
	
	QCBOREncode_OpenMapInMapSZ(&EncodeCtx, "speed_left");
	m_speed_left->getConfiguration( EncodeCtx);
	QCBOREncode_CloseMap(&EncodeCtx);
	

	QCBOREncode_CloseMap(&EncodeCtx);
}
