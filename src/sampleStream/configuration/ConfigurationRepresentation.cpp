#include "ConfigurationRepresentation.h"
#include <ch.h>
#include <hal.h>

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


void ConfigurationRepresentation::applyConfiguration(QCBORDecodeContext &decodeCtx)
{
	QCBORDecode_EnterMap(&decodeCtx, NULL);
		
	UsefulBufC name;
	QCBORDecode_GetTextStringInMapSZ(&decodeCtx, "name", &name);


	if ( strncmp((char*)name.ptr, "dist_acc" ,strlen("dist_acc")) == 0 )
	{
		m_dist_acc->applyConfiguration(decodeCtx);
	}
	else if ( strncmp((char*)name.ptr, "speed_left" ,strlen("speed_left")) == 0 )
	{
		m_speed_left->applyConfiguration(decodeCtx);
	}
	else if ( strncmp((char*)name.ptr, "speed_right" ,strlen("speed_right")) == 0 )
	{
		m_speed_right->applyConfiguration(decodeCtx);
	}
	else if ( strncmp((char*)name.ptr, "dist_regulator" ,strlen("dist_regulator")) == 0 )
	{
		m_dist_regulator->applyConfiguration(decodeCtx);
	}
	else if ( strncmp((char*)name.ptr, "angle_regulator" ,strlen("angle_regulator")) == 0 )
	{
		m_angle_regulator->applyConfiguration(decodeCtx);
	}
	else if ( strncmp((char*)name.ptr, "angle_acc" ,strlen("angle_acc")) == 0 )
	{
		m_angle_acc->applyConfiguration(decodeCtx);
	}
	
	QCBORDecode_ExitMap(&decodeCtx);
}