/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "E2SM-COMMON-IEs"
 * 	found in "e2sm-kpmv2.02.asn1"
 * 	`asn1c -gen-PER -no-gen-OER -fcompound-names -no-gen-example -findirect-choice -fno-include-deps -D asn`
 */

#include "NG-RANnodeUEXnAPID.h"

int
NG_RANnodeUEXnAPID_constraint(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	/* Constraint check succeeded */
	return 0;
}

/*
 * This type is implemented using NativeInteger,
 * so here we adjust the DEF accordingly.
 */
asn_per_constraints_t asn_PER_type_NG_RANnodeUEXnAPID_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 32, -1,  0,  4294967295 }	/* (0..4294967295) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
const asn_INTEGER_specifics_t asn_SPC_NG_RANnodeUEXnAPID_specs_1 = {
	0,	0,	0,	0,	0,
	0,	/* Native long size */
	1	/* Unsigned representation */
};
static const ber_tlv_tag_t asn_DEF_NG_RANnodeUEXnAPID_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (2 << 2))
};
asn_TYPE_descriptor_t asn_DEF_NG_RANnodeUEXnAPID = {
	"NG-RANnodeUEXnAPID",
	"NG-RANnodeUEXnAPID",
	&asn_OP_NativeInteger,
	asn_DEF_NG_RANnodeUEXnAPID_tags_1,
	sizeof(asn_DEF_NG_RANnodeUEXnAPID_tags_1)
		/sizeof(asn_DEF_NG_RANnodeUEXnAPID_tags_1[0]), /* 1 */
	asn_DEF_NG_RANnodeUEXnAPID_tags_1,	/* Same as above */
	sizeof(asn_DEF_NG_RANnodeUEXnAPID_tags_1)
		/sizeof(asn_DEF_NG_RANnodeUEXnAPID_tags_1[0]), /* 1 */
	{ 0, &asn_PER_type_NG_RANnodeUEXnAPID_constr_1, NG_RANnodeUEXnAPID_constraint },
	0, 0,	/* No members */
	&asn_SPC_NG_RANnodeUEXnAPID_specs_1	/* Additional specs */
};

