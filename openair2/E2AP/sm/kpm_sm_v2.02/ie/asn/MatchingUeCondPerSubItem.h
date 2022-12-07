/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "E2SM-KPM-IEs"
 * 	found in "e2sm-kpmv2.02.asn1"
 * 	`asn1c -gen-PER -no-gen-OER -fcompound-names -no-gen-example -findirect-choice -fno-include-deps -D asn`
 */

#ifndef	_MatchingUeCondPerSubItem_H_
#define	_MatchingUeCondPerSubItem_H_


#include <asn_application.h>

/* Including external dependencies */
#include "TestCondInfo.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MatchingUeCondPerSubItem */
typedef struct MatchingUeCondPerSubItem {
	TestCondInfo_t	 testCondInfo;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MatchingUeCondPerSubItem_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MatchingUeCondPerSubItem;
extern asn_SEQUENCE_specifics_t asn_SPC_MatchingUeCondPerSubItem_specs_1;
extern asn_TYPE_member_t asn_MBR_MatchingUeCondPerSubItem_1[1];

#ifdef __cplusplus
}
#endif

#endif	/* _MatchingUeCondPerSubItem_H_ */
#include <asn_internal.h>
