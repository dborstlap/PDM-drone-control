/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
int lRun2;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 12];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 12 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 12 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 12 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 12 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 12 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 12 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 12 + 7];
acadoWorkspace.state[8] = acadoVariables.x[lRun1 * 12 + 8];
acadoWorkspace.state[9] = acadoVariables.x[lRun1 * 12 + 9];
acadoWorkspace.state[10] = acadoVariables.x[lRun1 * 12 + 10];
acadoWorkspace.state[11] = acadoVariables.x[lRun1 * 12 + 11];

acadoWorkspace.state[204] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.state[205] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.state[206] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.state[207] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.state[208] = acadoVariables.od[lRun1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 12] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 12 + 12];
acadoWorkspace.d[lRun1 * 12 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 12 + 13];
acadoWorkspace.d[lRun1 * 12 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 12 + 14];
acadoWorkspace.d[lRun1 * 12 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 12 + 15];
acadoWorkspace.d[lRun1 * 12 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 12 + 16];
acadoWorkspace.d[lRun1 * 12 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 12 + 17];
acadoWorkspace.d[lRun1 * 12 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 12 + 18];
acadoWorkspace.d[lRun1 * 12 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 12 + 19];
acadoWorkspace.d[lRun1 * 12 + 8] = acadoWorkspace.state[8] - acadoVariables.x[lRun1 * 12 + 20];
acadoWorkspace.d[lRun1 * 12 + 9] = acadoWorkspace.state[9] - acadoVariables.x[lRun1 * 12 + 21];
acadoWorkspace.d[lRun1 * 12 + 10] = acadoWorkspace.state[10] - acadoVariables.x[lRun1 * 12 + 22];
acadoWorkspace.d[lRun1 * 12 + 11] = acadoWorkspace.state[11] - acadoVariables.x[lRun1 * 12 + 23];

for (lRun2 = 0; lRun2 < 144; ++lRun2)
acadoWorkspace.evGx[(0) + ((lRun2) + (lRun1 * 144))] = acadoWorkspace.state[lRun2 + 12];


acadoWorkspace.evGu[lRun1 * 48] = acadoWorkspace.state[156];
acadoWorkspace.evGu[lRun1 * 48 + 1] = acadoWorkspace.state[157];
acadoWorkspace.evGu[lRun1 * 48 + 2] = acadoWorkspace.state[158];
acadoWorkspace.evGu[lRun1 * 48 + 3] = acadoWorkspace.state[159];
acadoWorkspace.evGu[lRun1 * 48 + 4] = acadoWorkspace.state[160];
acadoWorkspace.evGu[lRun1 * 48 + 5] = acadoWorkspace.state[161];
acadoWorkspace.evGu[lRun1 * 48 + 6] = acadoWorkspace.state[162];
acadoWorkspace.evGu[lRun1 * 48 + 7] = acadoWorkspace.state[163];
acadoWorkspace.evGu[lRun1 * 48 + 8] = acadoWorkspace.state[164];
acadoWorkspace.evGu[lRun1 * 48 + 9] = acadoWorkspace.state[165];
acadoWorkspace.evGu[lRun1 * 48 + 10] = acadoWorkspace.state[166];
acadoWorkspace.evGu[lRun1 * 48 + 11] = acadoWorkspace.state[167];
acadoWorkspace.evGu[lRun1 * 48 + 12] = acadoWorkspace.state[168];
acadoWorkspace.evGu[lRun1 * 48 + 13] = acadoWorkspace.state[169];
acadoWorkspace.evGu[lRun1 * 48 + 14] = acadoWorkspace.state[170];
acadoWorkspace.evGu[lRun1 * 48 + 15] = acadoWorkspace.state[171];
acadoWorkspace.evGu[lRun1 * 48 + 16] = acadoWorkspace.state[172];
acadoWorkspace.evGu[lRun1 * 48 + 17] = acadoWorkspace.state[173];
acadoWorkspace.evGu[lRun1 * 48 + 18] = acadoWorkspace.state[174];
acadoWorkspace.evGu[lRun1 * 48 + 19] = acadoWorkspace.state[175];
acadoWorkspace.evGu[lRun1 * 48 + 20] = acadoWorkspace.state[176];
acadoWorkspace.evGu[lRun1 * 48 + 21] = acadoWorkspace.state[177];
acadoWorkspace.evGu[lRun1 * 48 + 22] = acadoWorkspace.state[178];
acadoWorkspace.evGu[lRun1 * 48 + 23] = acadoWorkspace.state[179];
acadoWorkspace.evGu[lRun1 * 48 + 24] = acadoWorkspace.state[180];
acadoWorkspace.evGu[lRun1 * 48 + 25] = acadoWorkspace.state[181];
acadoWorkspace.evGu[lRun1 * 48 + 26] = acadoWorkspace.state[182];
acadoWorkspace.evGu[lRun1 * 48 + 27] = acadoWorkspace.state[183];
acadoWorkspace.evGu[lRun1 * 48 + 28] = acadoWorkspace.state[184];
acadoWorkspace.evGu[lRun1 * 48 + 29] = acadoWorkspace.state[185];
acadoWorkspace.evGu[lRun1 * 48 + 30] = acadoWorkspace.state[186];
acadoWorkspace.evGu[lRun1 * 48 + 31] = acadoWorkspace.state[187];
acadoWorkspace.evGu[lRun1 * 48 + 32] = acadoWorkspace.state[188];
acadoWorkspace.evGu[lRun1 * 48 + 33] = acadoWorkspace.state[189];
acadoWorkspace.evGu[lRun1 * 48 + 34] = acadoWorkspace.state[190];
acadoWorkspace.evGu[lRun1 * 48 + 35] = acadoWorkspace.state[191];
acadoWorkspace.evGu[lRun1 * 48 + 36] = acadoWorkspace.state[192];
acadoWorkspace.evGu[lRun1 * 48 + 37] = acadoWorkspace.state[193];
acadoWorkspace.evGu[lRun1 * 48 + 38] = acadoWorkspace.state[194];
acadoWorkspace.evGu[lRun1 * 48 + 39] = acadoWorkspace.state[195];
acadoWorkspace.evGu[lRun1 * 48 + 40] = acadoWorkspace.state[196];
acadoWorkspace.evGu[lRun1 * 48 + 41] = acadoWorkspace.state[197];
acadoWorkspace.evGu[lRun1 * 48 + 42] = acadoWorkspace.state[198];
acadoWorkspace.evGu[lRun1 * 48 + 43] = acadoWorkspace.state[199];
acadoWorkspace.evGu[lRun1 * 48 + 44] = acadoWorkspace.state[200];
acadoWorkspace.evGu[lRun1 * 48 + 45] = acadoWorkspace.state[201];
acadoWorkspace.evGu[lRun1 * 48 + 46] = acadoWorkspace.state[202];
acadoWorkspace.evGu[lRun1 * 48 + 47] = acadoWorkspace.state[203];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 12;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = u[0];
out[7] = u[1];
out[8] = u[2];
out[9] = u[3];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = +tmpObjS[48];
tmpQ2[49] = +tmpObjS[49];
tmpQ2[50] = +tmpObjS[50];
tmpQ2[51] = +tmpObjS[51];
tmpQ2[52] = +tmpObjS[52];
tmpQ2[53] = +tmpObjS[53];
tmpQ2[54] = +tmpObjS[54];
tmpQ2[55] = +tmpObjS[55];
tmpQ2[56] = +tmpObjS[56];
tmpQ2[57] = +tmpObjS[57];
tmpQ2[58] = +tmpObjS[58];
tmpQ2[59] = +tmpObjS[59];
tmpQ2[60] = 0.0;
;
tmpQ2[61] = 0.0;
;
tmpQ2[62] = 0.0;
;
tmpQ2[63] = 0.0;
;
tmpQ2[64] = 0.0;
;
tmpQ2[65] = 0.0;
;
tmpQ2[66] = 0.0;
;
tmpQ2[67] = 0.0;
;
tmpQ2[68] = 0.0;
;
tmpQ2[69] = 0.0;
;
tmpQ2[70] = 0.0;
;
tmpQ2[71] = 0.0;
;
tmpQ2[72] = 0.0;
;
tmpQ2[73] = 0.0;
;
tmpQ2[74] = 0.0;
;
tmpQ2[75] = 0.0;
;
tmpQ2[76] = 0.0;
;
tmpQ2[77] = 0.0;
;
tmpQ2[78] = 0.0;
;
tmpQ2[79] = 0.0;
;
tmpQ2[80] = 0.0;
;
tmpQ2[81] = 0.0;
;
tmpQ2[82] = 0.0;
;
tmpQ2[83] = 0.0;
;
tmpQ2[84] = 0.0;
;
tmpQ2[85] = 0.0;
;
tmpQ2[86] = 0.0;
;
tmpQ2[87] = 0.0;
;
tmpQ2[88] = 0.0;
;
tmpQ2[89] = 0.0;
;
tmpQ2[90] = 0.0;
;
tmpQ2[91] = 0.0;
;
tmpQ2[92] = 0.0;
;
tmpQ2[93] = 0.0;
;
tmpQ2[94] = 0.0;
;
tmpQ2[95] = 0.0;
;
tmpQ2[96] = 0.0;
;
tmpQ2[97] = 0.0;
;
tmpQ2[98] = 0.0;
;
tmpQ2[99] = 0.0;
;
tmpQ2[100] = 0.0;
;
tmpQ2[101] = 0.0;
;
tmpQ2[102] = 0.0;
;
tmpQ2[103] = 0.0;
;
tmpQ2[104] = 0.0;
;
tmpQ2[105] = 0.0;
;
tmpQ2[106] = 0.0;
;
tmpQ2[107] = 0.0;
;
tmpQ2[108] = 0.0;
;
tmpQ2[109] = 0.0;
;
tmpQ2[110] = 0.0;
;
tmpQ2[111] = 0.0;
;
tmpQ2[112] = 0.0;
;
tmpQ2[113] = 0.0;
;
tmpQ2[114] = 0.0;
;
tmpQ2[115] = 0.0;
;
tmpQ2[116] = 0.0;
;
tmpQ2[117] = 0.0;
;
tmpQ2[118] = 0.0;
;
tmpQ2[119] = 0.0;
;
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = 0.0;
;
tmpQ1[7] = 0.0;
;
tmpQ1[8] = 0.0;
;
tmpQ1[9] = 0.0;
;
tmpQ1[10] = 0.0;
;
tmpQ1[11] = 0.0;
;
tmpQ1[12] = + tmpQ2[10];
tmpQ1[13] = + tmpQ2[11];
tmpQ1[14] = + tmpQ2[12];
tmpQ1[15] = + tmpQ2[13];
tmpQ1[16] = + tmpQ2[14];
tmpQ1[17] = + tmpQ2[15];
tmpQ1[18] = 0.0;
;
tmpQ1[19] = 0.0;
;
tmpQ1[20] = 0.0;
;
tmpQ1[21] = 0.0;
;
tmpQ1[22] = 0.0;
;
tmpQ1[23] = 0.0;
;
tmpQ1[24] = + tmpQ2[20];
tmpQ1[25] = + tmpQ2[21];
tmpQ1[26] = + tmpQ2[22];
tmpQ1[27] = + tmpQ2[23];
tmpQ1[28] = + tmpQ2[24];
tmpQ1[29] = + tmpQ2[25];
tmpQ1[30] = 0.0;
;
tmpQ1[31] = 0.0;
;
tmpQ1[32] = 0.0;
;
tmpQ1[33] = 0.0;
;
tmpQ1[34] = 0.0;
;
tmpQ1[35] = 0.0;
;
tmpQ1[36] = + tmpQ2[30];
tmpQ1[37] = + tmpQ2[31];
tmpQ1[38] = + tmpQ2[32];
tmpQ1[39] = + tmpQ2[33];
tmpQ1[40] = + tmpQ2[34];
tmpQ1[41] = + tmpQ2[35];
tmpQ1[42] = 0.0;
;
tmpQ1[43] = 0.0;
;
tmpQ1[44] = 0.0;
;
tmpQ1[45] = 0.0;
;
tmpQ1[46] = 0.0;
;
tmpQ1[47] = 0.0;
;
tmpQ1[48] = + tmpQ2[40];
tmpQ1[49] = + tmpQ2[41];
tmpQ1[50] = + tmpQ2[42];
tmpQ1[51] = + tmpQ2[43];
tmpQ1[52] = + tmpQ2[44];
tmpQ1[53] = + tmpQ2[45];
tmpQ1[54] = 0.0;
;
tmpQ1[55] = 0.0;
;
tmpQ1[56] = 0.0;
;
tmpQ1[57] = 0.0;
;
tmpQ1[58] = 0.0;
;
tmpQ1[59] = 0.0;
;
tmpQ1[60] = + tmpQ2[50];
tmpQ1[61] = + tmpQ2[51];
tmpQ1[62] = + tmpQ2[52];
tmpQ1[63] = + tmpQ2[53];
tmpQ1[64] = + tmpQ2[54];
tmpQ1[65] = + tmpQ2[55];
tmpQ1[66] = 0.0;
;
tmpQ1[67] = 0.0;
;
tmpQ1[68] = 0.0;
;
tmpQ1[69] = 0.0;
;
tmpQ1[70] = 0.0;
;
tmpQ1[71] = 0.0;
;
tmpQ1[72] = + tmpQ2[60];
tmpQ1[73] = + tmpQ2[61];
tmpQ1[74] = + tmpQ2[62];
tmpQ1[75] = + tmpQ2[63];
tmpQ1[76] = + tmpQ2[64];
tmpQ1[77] = + tmpQ2[65];
tmpQ1[78] = 0.0;
;
tmpQ1[79] = 0.0;
;
tmpQ1[80] = 0.0;
;
tmpQ1[81] = 0.0;
;
tmpQ1[82] = 0.0;
;
tmpQ1[83] = 0.0;
;
tmpQ1[84] = + tmpQ2[70];
tmpQ1[85] = + tmpQ2[71];
tmpQ1[86] = + tmpQ2[72];
tmpQ1[87] = + tmpQ2[73];
tmpQ1[88] = + tmpQ2[74];
tmpQ1[89] = + tmpQ2[75];
tmpQ1[90] = 0.0;
;
tmpQ1[91] = 0.0;
;
tmpQ1[92] = 0.0;
;
tmpQ1[93] = 0.0;
;
tmpQ1[94] = 0.0;
;
tmpQ1[95] = 0.0;
;
tmpQ1[96] = + tmpQ2[80];
tmpQ1[97] = + tmpQ2[81];
tmpQ1[98] = + tmpQ2[82];
tmpQ1[99] = + tmpQ2[83];
tmpQ1[100] = + tmpQ2[84];
tmpQ1[101] = + tmpQ2[85];
tmpQ1[102] = 0.0;
;
tmpQ1[103] = 0.0;
;
tmpQ1[104] = 0.0;
;
tmpQ1[105] = 0.0;
;
tmpQ1[106] = 0.0;
;
tmpQ1[107] = 0.0;
;
tmpQ1[108] = + tmpQ2[90];
tmpQ1[109] = + tmpQ2[91];
tmpQ1[110] = + tmpQ2[92];
tmpQ1[111] = + tmpQ2[93];
tmpQ1[112] = + tmpQ2[94];
tmpQ1[113] = + tmpQ2[95];
tmpQ1[114] = 0.0;
;
tmpQ1[115] = 0.0;
;
tmpQ1[116] = 0.0;
;
tmpQ1[117] = 0.0;
;
tmpQ1[118] = 0.0;
;
tmpQ1[119] = 0.0;
;
tmpQ1[120] = + tmpQ2[100];
tmpQ1[121] = + tmpQ2[101];
tmpQ1[122] = + tmpQ2[102];
tmpQ1[123] = + tmpQ2[103];
tmpQ1[124] = + tmpQ2[104];
tmpQ1[125] = + tmpQ2[105];
tmpQ1[126] = 0.0;
;
tmpQ1[127] = 0.0;
;
tmpQ1[128] = 0.0;
;
tmpQ1[129] = 0.0;
;
tmpQ1[130] = 0.0;
;
tmpQ1[131] = 0.0;
;
tmpQ1[132] = + tmpQ2[110];
tmpQ1[133] = + tmpQ2[111];
tmpQ1[134] = + tmpQ2[112];
tmpQ1[135] = + tmpQ2[113];
tmpQ1[136] = + tmpQ2[114];
tmpQ1[137] = + tmpQ2[115];
tmpQ1[138] = 0.0;
;
tmpQ1[139] = 0.0;
;
tmpQ1[140] = 0.0;
;
tmpQ1[141] = 0.0;
;
tmpQ1[142] = 0.0;
;
tmpQ1[143] = 0.0;
;
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[60];
tmpR2[1] = +tmpObjS[61];
tmpR2[2] = +tmpObjS[62];
tmpR2[3] = +tmpObjS[63];
tmpR2[4] = +tmpObjS[64];
tmpR2[5] = +tmpObjS[65];
tmpR2[6] = +tmpObjS[66];
tmpR2[7] = +tmpObjS[67];
tmpR2[8] = +tmpObjS[68];
tmpR2[9] = +tmpObjS[69];
tmpR2[10] = +tmpObjS[70];
tmpR2[11] = +tmpObjS[71];
tmpR2[12] = +tmpObjS[72];
tmpR2[13] = +tmpObjS[73];
tmpR2[14] = +tmpObjS[74];
tmpR2[15] = +tmpObjS[75];
tmpR2[16] = +tmpObjS[76];
tmpR2[17] = +tmpObjS[77];
tmpR2[18] = +tmpObjS[78];
tmpR2[19] = +tmpObjS[79];
tmpR2[20] = +tmpObjS[80];
tmpR2[21] = +tmpObjS[81];
tmpR2[22] = +tmpObjS[82];
tmpR2[23] = +tmpObjS[83];
tmpR2[24] = +tmpObjS[84];
tmpR2[25] = +tmpObjS[85];
tmpR2[26] = +tmpObjS[86];
tmpR2[27] = +tmpObjS[87];
tmpR2[28] = +tmpObjS[88];
tmpR2[29] = +tmpObjS[89];
tmpR2[30] = +tmpObjS[90];
tmpR2[31] = +tmpObjS[91];
tmpR2[32] = +tmpObjS[92];
tmpR2[33] = +tmpObjS[93];
tmpR2[34] = +tmpObjS[94];
tmpR2[35] = +tmpObjS[95];
tmpR2[36] = +tmpObjS[96];
tmpR2[37] = +tmpObjS[97];
tmpR2[38] = +tmpObjS[98];
tmpR2[39] = +tmpObjS[99];
tmpR1[0] = + tmpR2[6];
tmpR1[1] = + tmpR2[7];
tmpR1[2] = + tmpR2[8];
tmpR1[3] = + tmpR2[9];
tmpR1[4] = + tmpR2[16];
tmpR1[5] = + tmpR2[17];
tmpR1[6] = + tmpR2[18];
tmpR1[7] = + tmpR2[19];
tmpR1[8] = + tmpR2[26];
tmpR1[9] = + tmpR2[27];
tmpR1[10] = + tmpR2[28];
tmpR1[11] = + tmpR2[29];
tmpR1[12] = + tmpR2[36];
tmpR1[13] = + tmpR2[37];
tmpR1[14] = + tmpR2[38];
tmpR1[15] = + tmpR2[39];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = 0.0;
;
tmpQN2[37] = 0.0;
;
tmpQN2[38] = 0.0;
;
tmpQN2[39] = 0.0;
;
tmpQN2[40] = 0.0;
;
tmpQN2[41] = 0.0;
;
tmpQN2[42] = 0.0;
;
tmpQN2[43] = 0.0;
;
tmpQN2[44] = 0.0;
;
tmpQN2[45] = 0.0;
;
tmpQN2[46] = 0.0;
;
tmpQN2[47] = 0.0;
;
tmpQN2[48] = 0.0;
;
tmpQN2[49] = 0.0;
;
tmpQN2[50] = 0.0;
;
tmpQN2[51] = 0.0;
;
tmpQN2[52] = 0.0;
;
tmpQN2[53] = 0.0;
;
tmpQN2[54] = 0.0;
;
tmpQN2[55] = 0.0;
;
tmpQN2[56] = 0.0;
;
tmpQN2[57] = 0.0;
;
tmpQN2[58] = 0.0;
;
tmpQN2[59] = 0.0;
;
tmpQN2[60] = 0.0;
;
tmpQN2[61] = 0.0;
;
tmpQN2[62] = 0.0;
;
tmpQN2[63] = 0.0;
;
tmpQN2[64] = 0.0;
;
tmpQN2[65] = 0.0;
;
tmpQN2[66] = 0.0;
;
tmpQN2[67] = 0.0;
;
tmpQN2[68] = 0.0;
;
tmpQN2[69] = 0.0;
;
tmpQN2[70] = 0.0;
;
tmpQN2[71] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = 0.0;
;
tmpQN1[7] = 0.0;
;
tmpQN1[8] = 0.0;
;
tmpQN1[9] = 0.0;
;
tmpQN1[10] = 0.0;
;
tmpQN1[11] = 0.0;
;
tmpQN1[12] = + tmpQN2[6];
tmpQN1[13] = + tmpQN2[7];
tmpQN1[14] = + tmpQN2[8];
tmpQN1[15] = + tmpQN2[9];
tmpQN1[16] = + tmpQN2[10];
tmpQN1[17] = + tmpQN2[11];
tmpQN1[18] = 0.0;
;
tmpQN1[19] = 0.0;
;
tmpQN1[20] = 0.0;
;
tmpQN1[21] = 0.0;
;
tmpQN1[22] = 0.0;
;
tmpQN1[23] = 0.0;
;
tmpQN1[24] = + tmpQN2[12];
tmpQN1[25] = + tmpQN2[13];
tmpQN1[26] = + tmpQN2[14];
tmpQN1[27] = + tmpQN2[15];
tmpQN1[28] = + tmpQN2[16];
tmpQN1[29] = + tmpQN2[17];
tmpQN1[30] = 0.0;
;
tmpQN1[31] = 0.0;
;
tmpQN1[32] = 0.0;
;
tmpQN1[33] = 0.0;
;
tmpQN1[34] = 0.0;
;
tmpQN1[35] = 0.0;
;
tmpQN1[36] = + tmpQN2[18];
tmpQN1[37] = + tmpQN2[19];
tmpQN1[38] = + tmpQN2[20];
tmpQN1[39] = + tmpQN2[21];
tmpQN1[40] = + tmpQN2[22];
tmpQN1[41] = + tmpQN2[23];
tmpQN1[42] = 0.0;
;
tmpQN1[43] = 0.0;
;
tmpQN1[44] = 0.0;
;
tmpQN1[45] = 0.0;
;
tmpQN1[46] = 0.0;
;
tmpQN1[47] = 0.0;
;
tmpQN1[48] = + tmpQN2[24];
tmpQN1[49] = + tmpQN2[25];
tmpQN1[50] = + tmpQN2[26];
tmpQN1[51] = + tmpQN2[27];
tmpQN1[52] = + tmpQN2[28];
tmpQN1[53] = + tmpQN2[29];
tmpQN1[54] = 0.0;
;
tmpQN1[55] = 0.0;
;
tmpQN1[56] = 0.0;
;
tmpQN1[57] = 0.0;
;
tmpQN1[58] = 0.0;
;
tmpQN1[59] = 0.0;
;
tmpQN1[60] = + tmpQN2[30];
tmpQN1[61] = + tmpQN2[31];
tmpQN1[62] = + tmpQN2[32];
tmpQN1[63] = + tmpQN2[33];
tmpQN1[64] = + tmpQN2[34];
tmpQN1[65] = + tmpQN2[35];
tmpQN1[66] = 0.0;
;
tmpQN1[67] = 0.0;
;
tmpQN1[68] = 0.0;
;
tmpQN1[69] = 0.0;
;
tmpQN1[70] = 0.0;
;
tmpQN1[71] = 0.0;
;
tmpQN1[72] = + tmpQN2[36];
tmpQN1[73] = + tmpQN2[37];
tmpQN1[74] = + tmpQN2[38];
tmpQN1[75] = + tmpQN2[39];
tmpQN1[76] = + tmpQN2[40];
tmpQN1[77] = + tmpQN2[41];
tmpQN1[78] = 0.0;
;
tmpQN1[79] = 0.0;
;
tmpQN1[80] = 0.0;
;
tmpQN1[81] = 0.0;
;
tmpQN1[82] = 0.0;
;
tmpQN1[83] = 0.0;
;
tmpQN1[84] = + tmpQN2[42];
tmpQN1[85] = + tmpQN2[43];
tmpQN1[86] = + tmpQN2[44];
tmpQN1[87] = + tmpQN2[45];
tmpQN1[88] = + tmpQN2[46];
tmpQN1[89] = + tmpQN2[47];
tmpQN1[90] = 0.0;
;
tmpQN1[91] = 0.0;
;
tmpQN1[92] = 0.0;
;
tmpQN1[93] = 0.0;
;
tmpQN1[94] = 0.0;
;
tmpQN1[95] = 0.0;
;
tmpQN1[96] = + tmpQN2[48];
tmpQN1[97] = + tmpQN2[49];
tmpQN1[98] = + tmpQN2[50];
tmpQN1[99] = + tmpQN2[51];
tmpQN1[100] = + tmpQN2[52];
tmpQN1[101] = + tmpQN2[53];
tmpQN1[102] = 0.0;
;
tmpQN1[103] = 0.0;
;
tmpQN1[104] = 0.0;
;
tmpQN1[105] = 0.0;
;
tmpQN1[106] = 0.0;
;
tmpQN1[107] = 0.0;
;
tmpQN1[108] = + tmpQN2[54];
tmpQN1[109] = + tmpQN2[55];
tmpQN1[110] = + tmpQN2[56];
tmpQN1[111] = + tmpQN2[57];
tmpQN1[112] = + tmpQN2[58];
tmpQN1[113] = + tmpQN2[59];
tmpQN1[114] = 0.0;
;
tmpQN1[115] = 0.0;
;
tmpQN1[116] = 0.0;
;
tmpQN1[117] = 0.0;
;
tmpQN1[118] = 0.0;
;
tmpQN1[119] = 0.0;
;
tmpQN1[120] = + tmpQN2[60];
tmpQN1[121] = + tmpQN2[61];
tmpQN1[122] = + tmpQN2[62];
tmpQN1[123] = + tmpQN2[63];
tmpQN1[124] = + tmpQN2[64];
tmpQN1[125] = + tmpQN2[65];
tmpQN1[126] = 0.0;
;
tmpQN1[127] = 0.0;
;
tmpQN1[128] = 0.0;
;
tmpQN1[129] = 0.0;
;
tmpQN1[130] = 0.0;
;
tmpQN1[131] = 0.0;
;
tmpQN1[132] = + tmpQN2[66];
tmpQN1[133] = + tmpQN2[67];
tmpQN1[134] = + tmpQN2[68];
tmpQN1[135] = + tmpQN2[69];
tmpQN1[136] = + tmpQN2[70];
tmpQN1[137] = + tmpQN2[71];
tmpQN1[138] = 0.0;
;
tmpQN1[139] = 0.0;
;
tmpQN1[140] = 0.0;
;
tmpQN1[141] = 0.0;
;
tmpQN1[142] = 0.0;
;
tmpQN1[143] = 0.0;
;
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 12];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 12 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 12 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 12 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 12 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 12 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 12 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 12 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 12 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 12 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[runObj * 12 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[runObj * 12 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[13] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.u[runObj * 4 + 3];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 10] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 10 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 10 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 10 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 10 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 10 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 10 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 10 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 10 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 10 + 9] = acadoWorkspace.objValueOut[9];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 100 ]), &(acadoWorkspace.Q1[ runObj * 144 ]), &(acadoWorkspace.Q2[ runObj * 120 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 100 ]), &(acadoWorkspace.R1[ runObj * 16 ]), &(acadoWorkspace.R2[ runObj * 40 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[360];
acadoWorkspace.objValueIn[1] = acadoVariables.x[361];
acadoWorkspace.objValueIn[2] = acadoVariables.x[362];
acadoWorkspace.objValueIn[3] = acadoVariables.x[363];
acadoWorkspace.objValueIn[4] = acadoVariables.x[364];
acadoWorkspace.objValueIn[5] = acadoVariables.x[365];
acadoWorkspace.objValueIn[6] = acadoVariables.x[366];
acadoWorkspace.objValueIn[7] = acadoVariables.x[367];
acadoWorkspace.objValueIn[8] = acadoVariables.x[368];
acadoWorkspace.objValueIn[9] = acadoVariables.x[369];
acadoWorkspace.objValueIn[10] = acadoVariables.x[370];
acadoWorkspace.objValueIn[11] = acadoVariables.x[371];
acadoWorkspace.objValueIn[12] = acadoVariables.od[30];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8] + Gx1[9]*dOld[9] + Gx1[10]*dOld[10] + Gx1[11]*dOld[11];
dNew[1] += + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3] + Gx1[16]*dOld[4] + Gx1[17]*dOld[5] + Gx1[18]*dOld[6] + Gx1[19]*dOld[7] + Gx1[20]*dOld[8] + Gx1[21]*dOld[9] + Gx1[22]*dOld[10] + Gx1[23]*dOld[11];
dNew[2] += + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7] + Gx1[32]*dOld[8] + Gx1[33]*dOld[9] + Gx1[34]*dOld[10] + Gx1[35]*dOld[11];
dNew[3] += + Gx1[36]*dOld[0] + Gx1[37]*dOld[1] + Gx1[38]*dOld[2] + Gx1[39]*dOld[3] + Gx1[40]*dOld[4] + Gx1[41]*dOld[5] + Gx1[42]*dOld[6] + Gx1[43]*dOld[7] + Gx1[44]*dOld[8] + Gx1[45]*dOld[9] + Gx1[46]*dOld[10] + Gx1[47]*dOld[11];
dNew[4] += + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7] + Gx1[56]*dOld[8] + Gx1[57]*dOld[9] + Gx1[58]*dOld[10] + Gx1[59]*dOld[11];
dNew[5] += + Gx1[60]*dOld[0] + Gx1[61]*dOld[1] + Gx1[62]*dOld[2] + Gx1[63]*dOld[3] + Gx1[64]*dOld[4] + Gx1[65]*dOld[5] + Gx1[66]*dOld[6] + Gx1[67]*dOld[7] + Gx1[68]*dOld[8] + Gx1[69]*dOld[9] + Gx1[70]*dOld[10] + Gx1[71]*dOld[11];
dNew[6] += + Gx1[72]*dOld[0] + Gx1[73]*dOld[1] + Gx1[74]*dOld[2] + Gx1[75]*dOld[3] + Gx1[76]*dOld[4] + Gx1[77]*dOld[5] + Gx1[78]*dOld[6] + Gx1[79]*dOld[7] + Gx1[80]*dOld[8] + Gx1[81]*dOld[9] + Gx1[82]*dOld[10] + Gx1[83]*dOld[11];
dNew[7] += + Gx1[84]*dOld[0] + Gx1[85]*dOld[1] + Gx1[86]*dOld[2] + Gx1[87]*dOld[3] + Gx1[88]*dOld[4] + Gx1[89]*dOld[5] + Gx1[90]*dOld[6] + Gx1[91]*dOld[7] + Gx1[92]*dOld[8] + Gx1[93]*dOld[9] + Gx1[94]*dOld[10] + Gx1[95]*dOld[11];
dNew[8] += + Gx1[96]*dOld[0] + Gx1[97]*dOld[1] + Gx1[98]*dOld[2] + Gx1[99]*dOld[3] + Gx1[100]*dOld[4] + Gx1[101]*dOld[5] + Gx1[102]*dOld[6] + Gx1[103]*dOld[7] + Gx1[104]*dOld[8] + Gx1[105]*dOld[9] + Gx1[106]*dOld[10] + Gx1[107]*dOld[11];
dNew[9] += + Gx1[108]*dOld[0] + Gx1[109]*dOld[1] + Gx1[110]*dOld[2] + Gx1[111]*dOld[3] + Gx1[112]*dOld[4] + Gx1[113]*dOld[5] + Gx1[114]*dOld[6] + Gx1[115]*dOld[7] + Gx1[116]*dOld[8] + Gx1[117]*dOld[9] + Gx1[118]*dOld[10] + Gx1[119]*dOld[11];
dNew[10] += + Gx1[120]*dOld[0] + Gx1[121]*dOld[1] + Gx1[122]*dOld[2] + Gx1[123]*dOld[3] + Gx1[124]*dOld[4] + Gx1[125]*dOld[5] + Gx1[126]*dOld[6] + Gx1[127]*dOld[7] + Gx1[128]*dOld[8] + Gx1[129]*dOld[9] + Gx1[130]*dOld[10] + Gx1[131]*dOld[11];
dNew[11] += + Gx1[132]*dOld[0] + Gx1[133]*dOld[1] + Gx1[134]*dOld[2] + Gx1[135]*dOld[3] + Gx1[136]*dOld[4] + Gx1[137]*dOld[5] + Gx1[138]*dOld[6] + Gx1[139]*dOld[7] + Gx1[140]*dOld[8] + Gx1[141]*dOld[9] + Gx1[142]*dOld[10] + Gx1[143]*dOld[11];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
int lRun1;
int lRun2;
for (lRun1 = 0;lRun1 < 12; ++lRun1)
for (lRun2 = 0;lRun2 < 12; ++lRun2)
Gx2[(lRun1 * 12) + (lRun2)] = Gx1[(lRun1 * 12) + (lRun2)];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[24] + Gx1[3]*Gx2[36] + Gx1[4]*Gx2[48] + Gx1[5]*Gx2[60] + Gx1[6]*Gx2[72] + Gx1[7]*Gx2[84] + Gx1[8]*Gx2[96] + Gx1[9]*Gx2[108] + Gx1[10]*Gx2[120] + Gx1[11]*Gx2[132];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[25] + Gx1[3]*Gx2[37] + Gx1[4]*Gx2[49] + Gx1[5]*Gx2[61] + Gx1[6]*Gx2[73] + Gx1[7]*Gx2[85] + Gx1[8]*Gx2[97] + Gx1[9]*Gx2[109] + Gx1[10]*Gx2[121] + Gx1[11]*Gx2[133];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[26] + Gx1[3]*Gx2[38] + Gx1[4]*Gx2[50] + Gx1[5]*Gx2[62] + Gx1[6]*Gx2[74] + Gx1[7]*Gx2[86] + Gx1[8]*Gx2[98] + Gx1[9]*Gx2[110] + Gx1[10]*Gx2[122] + Gx1[11]*Gx2[134];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[27] + Gx1[3]*Gx2[39] + Gx1[4]*Gx2[51] + Gx1[5]*Gx2[63] + Gx1[6]*Gx2[75] + Gx1[7]*Gx2[87] + Gx1[8]*Gx2[99] + Gx1[9]*Gx2[111] + Gx1[10]*Gx2[123] + Gx1[11]*Gx2[135];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[16] + Gx1[2]*Gx2[28] + Gx1[3]*Gx2[40] + Gx1[4]*Gx2[52] + Gx1[5]*Gx2[64] + Gx1[6]*Gx2[76] + Gx1[7]*Gx2[88] + Gx1[8]*Gx2[100] + Gx1[9]*Gx2[112] + Gx1[10]*Gx2[124] + Gx1[11]*Gx2[136];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[17] + Gx1[2]*Gx2[29] + Gx1[3]*Gx2[41] + Gx1[4]*Gx2[53] + Gx1[5]*Gx2[65] + Gx1[6]*Gx2[77] + Gx1[7]*Gx2[89] + Gx1[8]*Gx2[101] + Gx1[9]*Gx2[113] + Gx1[10]*Gx2[125] + Gx1[11]*Gx2[137];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[18] + Gx1[2]*Gx2[30] + Gx1[3]*Gx2[42] + Gx1[4]*Gx2[54] + Gx1[5]*Gx2[66] + Gx1[6]*Gx2[78] + Gx1[7]*Gx2[90] + Gx1[8]*Gx2[102] + Gx1[9]*Gx2[114] + Gx1[10]*Gx2[126] + Gx1[11]*Gx2[138];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[19] + Gx1[2]*Gx2[31] + Gx1[3]*Gx2[43] + Gx1[4]*Gx2[55] + Gx1[5]*Gx2[67] + Gx1[6]*Gx2[79] + Gx1[7]*Gx2[91] + Gx1[8]*Gx2[103] + Gx1[9]*Gx2[115] + Gx1[10]*Gx2[127] + Gx1[11]*Gx2[139];
Gx3[8] = + Gx1[0]*Gx2[8] + Gx1[1]*Gx2[20] + Gx1[2]*Gx2[32] + Gx1[3]*Gx2[44] + Gx1[4]*Gx2[56] + Gx1[5]*Gx2[68] + Gx1[6]*Gx2[80] + Gx1[7]*Gx2[92] + Gx1[8]*Gx2[104] + Gx1[9]*Gx2[116] + Gx1[10]*Gx2[128] + Gx1[11]*Gx2[140];
Gx3[9] = + Gx1[0]*Gx2[9] + Gx1[1]*Gx2[21] + Gx1[2]*Gx2[33] + Gx1[3]*Gx2[45] + Gx1[4]*Gx2[57] + Gx1[5]*Gx2[69] + Gx1[6]*Gx2[81] + Gx1[7]*Gx2[93] + Gx1[8]*Gx2[105] + Gx1[9]*Gx2[117] + Gx1[10]*Gx2[129] + Gx1[11]*Gx2[141];
Gx3[10] = + Gx1[0]*Gx2[10] + Gx1[1]*Gx2[22] + Gx1[2]*Gx2[34] + Gx1[3]*Gx2[46] + Gx1[4]*Gx2[58] + Gx1[5]*Gx2[70] + Gx1[6]*Gx2[82] + Gx1[7]*Gx2[94] + Gx1[8]*Gx2[106] + Gx1[9]*Gx2[118] + Gx1[10]*Gx2[130] + Gx1[11]*Gx2[142];
Gx3[11] = + Gx1[0]*Gx2[11] + Gx1[1]*Gx2[23] + Gx1[2]*Gx2[35] + Gx1[3]*Gx2[47] + Gx1[4]*Gx2[59] + Gx1[5]*Gx2[71] + Gx1[6]*Gx2[83] + Gx1[7]*Gx2[95] + Gx1[8]*Gx2[107] + Gx1[9]*Gx2[119] + Gx1[10]*Gx2[131] + Gx1[11]*Gx2[143];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[12] + Gx1[14]*Gx2[24] + Gx1[15]*Gx2[36] + Gx1[16]*Gx2[48] + Gx1[17]*Gx2[60] + Gx1[18]*Gx2[72] + Gx1[19]*Gx2[84] + Gx1[20]*Gx2[96] + Gx1[21]*Gx2[108] + Gx1[22]*Gx2[120] + Gx1[23]*Gx2[132];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[13] + Gx1[14]*Gx2[25] + Gx1[15]*Gx2[37] + Gx1[16]*Gx2[49] + Gx1[17]*Gx2[61] + Gx1[18]*Gx2[73] + Gx1[19]*Gx2[85] + Gx1[20]*Gx2[97] + Gx1[21]*Gx2[109] + Gx1[22]*Gx2[121] + Gx1[23]*Gx2[133];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[14] + Gx1[14]*Gx2[26] + Gx1[15]*Gx2[38] + Gx1[16]*Gx2[50] + Gx1[17]*Gx2[62] + Gx1[18]*Gx2[74] + Gx1[19]*Gx2[86] + Gx1[20]*Gx2[98] + Gx1[21]*Gx2[110] + Gx1[22]*Gx2[122] + Gx1[23]*Gx2[134];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[27] + Gx1[15]*Gx2[39] + Gx1[16]*Gx2[51] + Gx1[17]*Gx2[63] + Gx1[18]*Gx2[75] + Gx1[19]*Gx2[87] + Gx1[20]*Gx2[99] + Gx1[21]*Gx2[111] + Gx1[22]*Gx2[123] + Gx1[23]*Gx2[135];
Gx3[16] = + Gx1[12]*Gx2[4] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[28] + Gx1[15]*Gx2[40] + Gx1[16]*Gx2[52] + Gx1[17]*Gx2[64] + Gx1[18]*Gx2[76] + Gx1[19]*Gx2[88] + Gx1[20]*Gx2[100] + Gx1[21]*Gx2[112] + Gx1[22]*Gx2[124] + Gx1[23]*Gx2[136];
Gx3[17] = + Gx1[12]*Gx2[5] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[29] + Gx1[15]*Gx2[41] + Gx1[16]*Gx2[53] + Gx1[17]*Gx2[65] + Gx1[18]*Gx2[77] + Gx1[19]*Gx2[89] + Gx1[20]*Gx2[101] + Gx1[21]*Gx2[113] + Gx1[22]*Gx2[125] + Gx1[23]*Gx2[137];
Gx3[18] = + Gx1[12]*Gx2[6] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[30] + Gx1[15]*Gx2[42] + Gx1[16]*Gx2[54] + Gx1[17]*Gx2[66] + Gx1[18]*Gx2[78] + Gx1[19]*Gx2[90] + Gx1[20]*Gx2[102] + Gx1[21]*Gx2[114] + Gx1[22]*Gx2[126] + Gx1[23]*Gx2[138];
Gx3[19] = + Gx1[12]*Gx2[7] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[31] + Gx1[15]*Gx2[43] + Gx1[16]*Gx2[55] + Gx1[17]*Gx2[67] + Gx1[18]*Gx2[79] + Gx1[19]*Gx2[91] + Gx1[20]*Gx2[103] + Gx1[21]*Gx2[115] + Gx1[22]*Gx2[127] + Gx1[23]*Gx2[139];
Gx3[20] = + Gx1[12]*Gx2[8] + Gx1[13]*Gx2[20] + Gx1[14]*Gx2[32] + Gx1[15]*Gx2[44] + Gx1[16]*Gx2[56] + Gx1[17]*Gx2[68] + Gx1[18]*Gx2[80] + Gx1[19]*Gx2[92] + Gx1[20]*Gx2[104] + Gx1[21]*Gx2[116] + Gx1[22]*Gx2[128] + Gx1[23]*Gx2[140];
Gx3[21] = + Gx1[12]*Gx2[9] + Gx1[13]*Gx2[21] + Gx1[14]*Gx2[33] + Gx1[15]*Gx2[45] + Gx1[16]*Gx2[57] + Gx1[17]*Gx2[69] + Gx1[18]*Gx2[81] + Gx1[19]*Gx2[93] + Gx1[20]*Gx2[105] + Gx1[21]*Gx2[117] + Gx1[22]*Gx2[129] + Gx1[23]*Gx2[141];
Gx3[22] = + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[22] + Gx1[14]*Gx2[34] + Gx1[15]*Gx2[46] + Gx1[16]*Gx2[58] + Gx1[17]*Gx2[70] + Gx1[18]*Gx2[82] + Gx1[19]*Gx2[94] + Gx1[20]*Gx2[106] + Gx1[21]*Gx2[118] + Gx1[22]*Gx2[130] + Gx1[23]*Gx2[142];
Gx3[23] = + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[23] + Gx1[14]*Gx2[35] + Gx1[15]*Gx2[47] + Gx1[16]*Gx2[59] + Gx1[17]*Gx2[71] + Gx1[18]*Gx2[83] + Gx1[19]*Gx2[95] + Gx1[20]*Gx2[107] + Gx1[21]*Gx2[119] + Gx1[22]*Gx2[131] + Gx1[23]*Gx2[143];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[12] + Gx1[26]*Gx2[24] + Gx1[27]*Gx2[36] + Gx1[28]*Gx2[48] + Gx1[29]*Gx2[60] + Gx1[30]*Gx2[72] + Gx1[31]*Gx2[84] + Gx1[32]*Gx2[96] + Gx1[33]*Gx2[108] + Gx1[34]*Gx2[120] + Gx1[35]*Gx2[132];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[13] + Gx1[26]*Gx2[25] + Gx1[27]*Gx2[37] + Gx1[28]*Gx2[49] + Gx1[29]*Gx2[61] + Gx1[30]*Gx2[73] + Gx1[31]*Gx2[85] + Gx1[32]*Gx2[97] + Gx1[33]*Gx2[109] + Gx1[34]*Gx2[121] + Gx1[35]*Gx2[133];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[14] + Gx1[26]*Gx2[26] + Gx1[27]*Gx2[38] + Gx1[28]*Gx2[50] + Gx1[29]*Gx2[62] + Gx1[30]*Gx2[74] + Gx1[31]*Gx2[86] + Gx1[32]*Gx2[98] + Gx1[33]*Gx2[110] + Gx1[34]*Gx2[122] + Gx1[35]*Gx2[134];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[15] + Gx1[26]*Gx2[27] + Gx1[27]*Gx2[39] + Gx1[28]*Gx2[51] + Gx1[29]*Gx2[63] + Gx1[30]*Gx2[75] + Gx1[31]*Gx2[87] + Gx1[32]*Gx2[99] + Gx1[33]*Gx2[111] + Gx1[34]*Gx2[123] + Gx1[35]*Gx2[135];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[16] + Gx1[26]*Gx2[28] + Gx1[27]*Gx2[40] + Gx1[28]*Gx2[52] + Gx1[29]*Gx2[64] + Gx1[30]*Gx2[76] + Gx1[31]*Gx2[88] + Gx1[32]*Gx2[100] + Gx1[33]*Gx2[112] + Gx1[34]*Gx2[124] + Gx1[35]*Gx2[136];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[17] + Gx1[26]*Gx2[29] + Gx1[27]*Gx2[41] + Gx1[28]*Gx2[53] + Gx1[29]*Gx2[65] + Gx1[30]*Gx2[77] + Gx1[31]*Gx2[89] + Gx1[32]*Gx2[101] + Gx1[33]*Gx2[113] + Gx1[34]*Gx2[125] + Gx1[35]*Gx2[137];
Gx3[30] = + Gx1[24]*Gx2[6] + Gx1[25]*Gx2[18] + Gx1[26]*Gx2[30] + Gx1[27]*Gx2[42] + Gx1[28]*Gx2[54] + Gx1[29]*Gx2[66] + Gx1[30]*Gx2[78] + Gx1[31]*Gx2[90] + Gx1[32]*Gx2[102] + Gx1[33]*Gx2[114] + Gx1[34]*Gx2[126] + Gx1[35]*Gx2[138];
Gx3[31] = + Gx1[24]*Gx2[7] + Gx1[25]*Gx2[19] + Gx1[26]*Gx2[31] + Gx1[27]*Gx2[43] + Gx1[28]*Gx2[55] + Gx1[29]*Gx2[67] + Gx1[30]*Gx2[79] + Gx1[31]*Gx2[91] + Gx1[32]*Gx2[103] + Gx1[33]*Gx2[115] + Gx1[34]*Gx2[127] + Gx1[35]*Gx2[139];
Gx3[32] = + Gx1[24]*Gx2[8] + Gx1[25]*Gx2[20] + Gx1[26]*Gx2[32] + Gx1[27]*Gx2[44] + Gx1[28]*Gx2[56] + Gx1[29]*Gx2[68] + Gx1[30]*Gx2[80] + Gx1[31]*Gx2[92] + Gx1[32]*Gx2[104] + Gx1[33]*Gx2[116] + Gx1[34]*Gx2[128] + Gx1[35]*Gx2[140];
Gx3[33] = + Gx1[24]*Gx2[9] + Gx1[25]*Gx2[21] + Gx1[26]*Gx2[33] + Gx1[27]*Gx2[45] + Gx1[28]*Gx2[57] + Gx1[29]*Gx2[69] + Gx1[30]*Gx2[81] + Gx1[31]*Gx2[93] + Gx1[32]*Gx2[105] + Gx1[33]*Gx2[117] + Gx1[34]*Gx2[129] + Gx1[35]*Gx2[141];
Gx3[34] = + Gx1[24]*Gx2[10] + Gx1[25]*Gx2[22] + Gx1[26]*Gx2[34] + Gx1[27]*Gx2[46] + Gx1[28]*Gx2[58] + Gx1[29]*Gx2[70] + Gx1[30]*Gx2[82] + Gx1[31]*Gx2[94] + Gx1[32]*Gx2[106] + Gx1[33]*Gx2[118] + Gx1[34]*Gx2[130] + Gx1[35]*Gx2[142];
Gx3[35] = + Gx1[24]*Gx2[11] + Gx1[25]*Gx2[23] + Gx1[26]*Gx2[35] + Gx1[27]*Gx2[47] + Gx1[28]*Gx2[59] + Gx1[29]*Gx2[71] + Gx1[30]*Gx2[83] + Gx1[31]*Gx2[95] + Gx1[32]*Gx2[107] + Gx1[33]*Gx2[119] + Gx1[34]*Gx2[131] + Gx1[35]*Gx2[143];
Gx3[36] = + Gx1[36]*Gx2[0] + Gx1[37]*Gx2[12] + Gx1[38]*Gx2[24] + Gx1[39]*Gx2[36] + Gx1[40]*Gx2[48] + Gx1[41]*Gx2[60] + Gx1[42]*Gx2[72] + Gx1[43]*Gx2[84] + Gx1[44]*Gx2[96] + Gx1[45]*Gx2[108] + Gx1[46]*Gx2[120] + Gx1[47]*Gx2[132];
Gx3[37] = + Gx1[36]*Gx2[1] + Gx1[37]*Gx2[13] + Gx1[38]*Gx2[25] + Gx1[39]*Gx2[37] + Gx1[40]*Gx2[49] + Gx1[41]*Gx2[61] + Gx1[42]*Gx2[73] + Gx1[43]*Gx2[85] + Gx1[44]*Gx2[97] + Gx1[45]*Gx2[109] + Gx1[46]*Gx2[121] + Gx1[47]*Gx2[133];
Gx3[38] = + Gx1[36]*Gx2[2] + Gx1[37]*Gx2[14] + Gx1[38]*Gx2[26] + Gx1[39]*Gx2[38] + Gx1[40]*Gx2[50] + Gx1[41]*Gx2[62] + Gx1[42]*Gx2[74] + Gx1[43]*Gx2[86] + Gx1[44]*Gx2[98] + Gx1[45]*Gx2[110] + Gx1[46]*Gx2[122] + Gx1[47]*Gx2[134];
Gx3[39] = + Gx1[36]*Gx2[3] + Gx1[37]*Gx2[15] + Gx1[38]*Gx2[27] + Gx1[39]*Gx2[39] + Gx1[40]*Gx2[51] + Gx1[41]*Gx2[63] + Gx1[42]*Gx2[75] + Gx1[43]*Gx2[87] + Gx1[44]*Gx2[99] + Gx1[45]*Gx2[111] + Gx1[46]*Gx2[123] + Gx1[47]*Gx2[135];
Gx3[40] = + Gx1[36]*Gx2[4] + Gx1[37]*Gx2[16] + Gx1[38]*Gx2[28] + Gx1[39]*Gx2[40] + Gx1[40]*Gx2[52] + Gx1[41]*Gx2[64] + Gx1[42]*Gx2[76] + Gx1[43]*Gx2[88] + Gx1[44]*Gx2[100] + Gx1[45]*Gx2[112] + Gx1[46]*Gx2[124] + Gx1[47]*Gx2[136];
Gx3[41] = + Gx1[36]*Gx2[5] + Gx1[37]*Gx2[17] + Gx1[38]*Gx2[29] + Gx1[39]*Gx2[41] + Gx1[40]*Gx2[53] + Gx1[41]*Gx2[65] + Gx1[42]*Gx2[77] + Gx1[43]*Gx2[89] + Gx1[44]*Gx2[101] + Gx1[45]*Gx2[113] + Gx1[46]*Gx2[125] + Gx1[47]*Gx2[137];
Gx3[42] = + Gx1[36]*Gx2[6] + Gx1[37]*Gx2[18] + Gx1[38]*Gx2[30] + Gx1[39]*Gx2[42] + Gx1[40]*Gx2[54] + Gx1[41]*Gx2[66] + Gx1[42]*Gx2[78] + Gx1[43]*Gx2[90] + Gx1[44]*Gx2[102] + Gx1[45]*Gx2[114] + Gx1[46]*Gx2[126] + Gx1[47]*Gx2[138];
Gx3[43] = + Gx1[36]*Gx2[7] + Gx1[37]*Gx2[19] + Gx1[38]*Gx2[31] + Gx1[39]*Gx2[43] + Gx1[40]*Gx2[55] + Gx1[41]*Gx2[67] + Gx1[42]*Gx2[79] + Gx1[43]*Gx2[91] + Gx1[44]*Gx2[103] + Gx1[45]*Gx2[115] + Gx1[46]*Gx2[127] + Gx1[47]*Gx2[139];
Gx3[44] = + Gx1[36]*Gx2[8] + Gx1[37]*Gx2[20] + Gx1[38]*Gx2[32] + Gx1[39]*Gx2[44] + Gx1[40]*Gx2[56] + Gx1[41]*Gx2[68] + Gx1[42]*Gx2[80] + Gx1[43]*Gx2[92] + Gx1[44]*Gx2[104] + Gx1[45]*Gx2[116] + Gx1[46]*Gx2[128] + Gx1[47]*Gx2[140];
Gx3[45] = + Gx1[36]*Gx2[9] + Gx1[37]*Gx2[21] + Gx1[38]*Gx2[33] + Gx1[39]*Gx2[45] + Gx1[40]*Gx2[57] + Gx1[41]*Gx2[69] + Gx1[42]*Gx2[81] + Gx1[43]*Gx2[93] + Gx1[44]*Gx2[105] + Gx1[45]*Gx2[117] + Gx1[46]*Gx2[129] + Gx1[47]*Gx2[141];
Gx3[46] = + Gx1[36]*Gx2[10] + Gx1[37]*Gx2[22] + Gx1[38]*Gx2[34] + Gx1[39]*Gx2[46] + Gx1[40]*Gx2[58] + Gx1[41]*Gx2[70] + Gx1[42]*Gx2[82] + Gx1[43]*Gx2[94] + Gx1[44]*Gx2[106] + Gx1[45]*Gx2[118] + Gx1[46]*Gx2[130] + Gx1[47]*Gx2[142];
Gx3[47] = + Gx1[36]*Gx2[11] + Gx1[37]*Gx2[23] + Gx1[38]*Gx2[35] + Gx1[39]*Gx2[47] + Gx1[40]*Gx2[59] + Gx1[41]*Gx2[71] + Gx1[42]*Gx2[83] + Gx1[43]*Gx2[95] + Gx1[44]*Gx2[107] + Gx1[45]*Gx2[119] + Gx1[46]*Gx2[131] + Gx1[47]*Gx2[143];
Gx3[48] = + Gx1[48]*Gx2[0] + Gx1[49]*Gx2[12] + Gx1[50]*Gx2[24] + Gx1[51]*Gx2[36] + Gx1[52]*Gx2[48] + Gx1[53]*Gx2[60] + Gx1[54]*Gx2[72] + Gx1[55]*Gx2[84] + Gx1[56]*Gx2[96] + Gx1[57]*Gx2[108] + Gx1[58]*Gx2[120] + Gx1[59]*Gx2[132];
Gx3[49] = + Gx1[48]*Gx2[1] + Gx1[49]*Gx2[13] + Gx1[50]*Gx2[25] + Gx1[51]*Gx2[37] + Gx1[52]*Gx2[49] + Gx1[53]*Gx2[61] + Gx1[54]*Gx2[73] + Gx1[55]*Gx2[85] + Gx1[56]*Gx2[97] + Gx1[57]*Gx2[109] + Gx1[58]*Gx2[121] + Gx1[59]*Gx2[133];
Gx3[50] = + Gx1[48]*Gx2[2] + Gx1[49]*Gx2[14] + Gx1[50]*Gx2[26] + Gx1[51]*Gx2[38] + Gx1[52]*Gx2[50] + Gx1[53]*Gx2[62] + Gx1[54]*Gx2[74] + Gx1[55]*Gx2[86] + Gx1[56]*Gx2[98] + Gx1[57]*Gx2[110] + Gx1[58]*Gx2[122] + Gx1[59]*Gx2[134];
Gx3[51] = + Gx1[48]*Gx2[3] + Gx1[49]*Gx2[15] + Gx1[50]*Gx2[27] + Gx1[51]*Gx2[39] + Gx1[52]*Gx2[51] + Gx1[53]*Gx2[63] + Gx1[54]*Gx2[75] + Gx1[55]*Gx2[87] + Gx1[56]*Gx2[99] + Gx1[57]*Gx2[111] + Gx1[58]*Gx2[123] + Gx1[59]*Gx2[135];
Gx3[52] = + Gx1[48]*Gx2[4] + Gx1[49]*Gx2[16] + Gx1[50]*Gx2[28] + Gx1[51]*Gx2[40] + Gx1[52]*Gx2[52] + Gx1[53]*Gx2[64] + Gx1[54]*Gx2[76] + Gx1[55]*Gx2[88] + Gx1[56]*Gx2[100] + Gx1[57]*Gx2[112] + Gx1[58]*Gx2[124] + Gx1[59]*Gx2[136];
Gx3[53] = + Gx1[48]*Gx2[5] + Gx1[49]*Gx2[17] + Gx1[50]*Gx2[29] + Gx1[51]*Gx2[41] + Gx1[52]*Gx2[53] + Gx1[53]*Gx2[65] + Gx1[54]*Gx2[77] + Gx1[55]*Gx2[89] + Gx1[56]*Gx2[101] + Gx1[57]*Gx2[113] + Gx1[58]*Gx2[125] + Gx1[59]*Gx2[137];
Gx3[54] = + Gx1[48]*Gx2[6] + Gx1[49]*Gx2[18] + Gx1[50]*Gx2[30] + Gx1[51]*Gx2[42] + Gx1[52]*Gx2[54] + Gx1[53]*Gx2[66] + Gx1[54]*Gx2[78] + Gx1[55]*Gx2[90] + Gx1[56]*Gx2[102] + Gx1[57]*Gx2[114] + Gx1[58]*Gx2[126] + Gx1[59]*Gx2[138];
Gx3[55] = + Gx1[48]*Gx2[7] + Gx1[49]*Gx2[19] + Gx1[50]*Gx2[31] + Gx1[51]*Gx2[43] + Gx1[52]*Gx2[55] + Gx1[53]*Gx2[67] + Gx1[54]*Gx2[79] + Gx1[55]*Gx2[91] + Gx1[56]*Gx2[103] + Gx1[57]*Gx2[115] + Gx1[58]*Gx2[127] + Gx1[59]*Gx2[139];
Gx3[56] = + Gx1[48]*Gx2[8] + Gx1[49]*Gx2[20] + Gx1[50]*Gx2[32] + Gx1[51]*Gx2[44] + Gx1[52]*Gx2[56] + Gx1[53]*Gx2[68] + Gx1[54]*Gx2[80] + Gx1[55]*Gx2[92] + Gx1[56]*Gx2[104] + Gx1[57]*Gx2[116] + Gx1[58]*Gx2[128] + Gx1[59]*Gx2[140];
Gx3[57] = + Gx1[48]*Gx2[9] + Gx1[49]*Gx2[21] + Gx1[50]*Gx2[33] + Gx1[51]*Gx2[45] + Gx1[52]*Gx2[57] + Gx1[53]*Gx2[69] + Gx1[54]*Gx2[81] + Gx1[55]*Gx2[93] + Gx1[56]*Gx2[105] + Gx1[57]*Gx2[117] + Gx1[58]*Gx2[129] + Gx1[59]*Gx2[141];
Gx3[58] = + Gx1[48]*Gx2[10] + Gx1[49]*Gx2[22] + Gx1[50]*Gx2[34] + Gx1[51]*Gx2[46] + Gx1[52]*Gx2[58] + Gx1[53]*Gx2[70] + Gx1[54]*Gx2[82] + Gx1[55]*Gx2[94] + Gx1[56]*Gx2[106] + Gx1[57]*Gx2[118] + Gx1[58]*Gx2[130] + Gx1[59]*Gx2[142];
Gx3[59] = + Gx1[48]*Gx2[11] + Gx1[49]*Gx2[23] + Gx1[50]*Gx2[35] + Gx1[51]*Gx2[47] + Gx1[52]*Gx2[59] + Gx1[53]*Gx2[71] + Gx1[54]*Gx2[83] + Gx1[55]*Gx2[95] + Gx1[56]*Gx2[107] + Gx1[57]*Gx2[119] + Gx1[58]*Gx2[131] + Gx1[59]*Gx2[143];
Gx3[60] = + Gx1[60]*Gx2[0] + Gx1[61]*Gx2[12] + Gx1[62]*Gx2[24] + Gx1[63]*Gx2[36] + Gx1[64]*Gx2[48] + Gx1[65]*Gx2[60] + Gx1[66]*Gx2[72] + Gx1[67]*Gx2[84] + Gx1[68]*Gx2[96] + Gx1[69]*Gx2[108] + Gx1[70]*Gx2[120] + Gx1[71]*Gx2[132];
Gx3[61] = + Gx1[60]*Gx2[1] + Gx1[61]*Gx2[13] + Gx1[62]*Gx2[25] + Gx1[63]*Gx2[37] + Gx1[64]*Gx2[49] + Gx1[65]*Gx2[61] + Gx1[66]*Gx2[73] + Gx1[67]*Gx2[85] + Gx1[68]*Gx2[97] + Gx1[69]*Gx2[109] + Gx1[70]*Gx2[121] + Gx1[71]*Gx2[133];
Gx3[62] = + Gx1[60]*Gx2[2] + Gx1[61]*Gx2[14] + Gx1[62]*Gx2[26] + Gx1[63]*Gx2[38] + Gx1[64]*Gx2[50] + Gx1[65]*Gx2[62] + Gx1[66]*Gx2[74] + Gx1[67]*Gx2[86] + Gx1[68]*Gx2[98] + Gx1[69]*Gx2[110] + Gx1[70]*Gx2[122] + Gx1[71]*Gx2[134];
Gx3[63] = + Gx1[60]*Gx2[3] + Gx1[61]*Gx2[15] + Gx1[62]*Gx2[27] + Gx1[63]*Gx2[39] + Gx1[64]*Gx2[51] + Gx1[65]*Gx2[63] + Gx1[66]*Gx2[75] + Gx1[67]*Gx2[87] + Gx1[68]*Gx2[99] + Gx1[69]*Gx2[111] + Gx1[70]*Gx2[123] + Gx1[71]*Gx2[135];
Gx3[64] = + Gx1[60]*Gx2[4] + Gx1[61]*Gx2[16] + Gx1[62]*Gx2[28] + Gx1[63]*Gx2[40] + Gx1[64]*Gx2[52] + Gx1[65]*Gx2[64] + Gx1[66]*Gx2[76] + Gx1[67]*Gx2[88] + Gx1[68]*Gx2[100] + Gx1[69]*Gx2[112] + Gx1[70]*Gx2[124] + Gx1[71]*Gx2[136];
Gx3[65] = + Gx1[60]*Gx2[5] + Gx1[61]*Gx2[17] + Gx1[62]*Gx2[29] + Gx1[63]*Gx2[41] + Gx1[64]*Gx2[53] + Gx1[65]*Gx2[65] + Gx1[66]*Gx2[77] + Gx1[67]*Gx2[89] + Gx1[68]*Gx2[101] + Gx1[69]*Gx2[113] + Gx1[70]*Gx2[125] + Gx1[71]*Gx2[137];
Gx3[66] = + Gx1[60]*Gx2[6] + Gx1[61]*Gx2[18] + Gx1[62]*Gx2[30] + Gx1[63]*Gx2[42] + Gx1[64]*Gx2[54] + Gx1[65]*Gx2[66] + Gx1[66]*Gx2[78] + Gx1[67]*Gx2[90] + Gx1[68]*Gx2[102] + Gx1[69]*Gx2[114] + Gx1[70]*Gx2[126] + Gx1[71]*Gx2[138];
Gx3[67] = + Gx1[60]*Gx2[7] + Gx1[61]*Gx2[19] + Gx1[62]*Gx2[31] + Gx1[63]*Gx2[43] + Gx1[64]*Gx2[55] + Gx1[65]*Gx2[67] + Gx1[66]*Gx2[79] + Gx1[67]*Gx2[91] + Gx1[68]*Gx2[103] + Gx1[69]*Gx2[115] + Gx1[70]*Gx2[127] + Gx1[71]*Gx2[139];
Gx3[68] = + Gx1[60]*Gx2[8] + Gx1[61]*Gx2[20] + Gx1[62]*Gx2[32] + Gx1[63]*Gx2[44] + Gx1[64]*Gx2[56] + Gx1[65]*Gx2[68] + Gx1[66]*Gx2[80] + Gx1[67]*Gx2[92] + Gx1[68]*Gx2[104] + Gx1[69]*Gx2[116] + Gx1[70]*Gx2[128] + Gx1[71]*Gx2[140];
Gx3[69] = + Gx1[60]*Gx2[9] + Gx1[61]*Gx2[21] + Gx1[62]*Gx2[33] + Gx1[63]*Gx2[45] + Gx1[64]*Gx2[57] + Gx1[65]*Gx2[69] + Gx1[66]*Gx2[81] + Gx1[67]*Gx2[93] + Gx1[68]*Gx2[105] + Gx1[69]*Gx2[117] + Gx1[70]*Gx2[129] + Gx1[71]*Gx2[141];
Gx3[70] = + Gx1[60]*Gx2[10] + Gx1[61]*Gx2[22] + Gx1[62]*Gx2[34] + Gx1[63]*Gx2[46] + Gx1[64]*Gx2[58] + Gx1[65]*Gx2[70] + Gx1[66]*Gx2[82] + Gx1[67]*Gx2[94] + Gx1[68]*Gx2[106] + Gx1[69]*Gx2[118] + Gx1[70]*Gx2[130] + Gx1[71]*Gx2[142];
Gx3[71] = + Gx1[60]*Gx2[11] + Gx1[61]*Gx2[23] + Gx1[62]*Gx2[35] + Gx1[63]*Gx2[47] + Gx1[64]*Gx2[59] + Gx1[65]*Gx2[71] + Gx1[66]*Gx2[83] + Gx1[67]*Gx2[95] + Gx1[68]*Gx2[107] + Gx1[69]*Gx2[119] + Gx1[70]*Gx2[131] + Gx1[71]*Gx2[143];
Gx3[72] = + Gx1[72]*Gx2[0] + Gx1[73]*Gx2[12] + Gx1[74]*Gx2[24] + Gx1[75]*Gx2[36] + Gx1[76]*Gx2[48] + Gx1[77]*Gx2[60] + Gx1[78]*Gx2[72] + Gx1[79]*Gx2[84] + Gx1[80]*Gx2[96] + Gx1[81]*Gx2[108] + Gx1[82]*Gx2[120] + Gx1[83]*Gx2[132];
Gx3[73] = + Gx1[72]*Gx2[1] + Gx1[73]*Gx2[13] + Gx1[74]*Gx2[25] + Gx1[75]*Gx2[37] + Gx1[76]*Gx2[49] + Gx1[77]*Gx2[61] + Gx1[78]*Gx2[73] + Gx1[79]*Gx2[85] + Gx1[80]*Gx2[97] + Gx1[81]*Gx2[109] + Gx1[82]*Gx2[121] + Gx1[83]*Gx2[133];
Gx3[74] = + Gx1[72]*Gx2[2] + Gx1[73]*Gx2[14] + Gx1[74]*Gx2[26] + Gx1[75]*Gx2[38] + Gx1[76]*Gx2[50] + Gx1[77]*Gx2[62] + Gx1[78]*Gx2[74] + Gx1[79]*Gx2[86] + Gx1[80]*Gx2[98] + Gx1[81]*Gx2[110] + Gx1[82]*Gx2[122] + Gx1[83]*Gx2[134];
Gx3[75] = + Gx1[72]*Gx2[3] + Gx1[73]*Gx2[15] + Gx1[74]*Gx2[27] + Gx1[75]*Gx2[39] + Gx1[76]*Gx2[51] + Gx1[77]*Gx2[63] + Gx1[78]*Gx2[75] + Gx1[79]*Gx2[87] + Gx1[80]*Gx2[99] + Gx1[81]*Gx2[111] + Gx1[82]*Gx2[123] + Gx1[83]*Gx2[135];
Gx3[76] = + Gx1[72]*Gx2[4] + Gx1[73]*Gx2[16] + Gx1[74]*Gx2[28] + Gx1[75]*Gx2[40] + Gx1[76]*Gx2[52] + Gx1[77]*Gx2[64] + Gx1[78]*Gx2[76] + Gx1[79]*Gx2[88] + Gx1[80]*Gx2[100] + Gx1[81]*Gx2[112] + Gx1[82]*Gx2[124] + Gx1[83]*Gx2[136];
Gx3[77] = + Gx1[72]*Gx2[5] + Gx1[73]*Gx2[17] + Gx1[74]*Gx2[29] + Gx1[75]*Gx2[41] + Gx1[76]*Gx2[53] + Gx1[77]*Gx2[65] + Gx1[78]*Gx2[77] + Gx1[79]*Gx2[89] + Gx1[80]*Gx2[101] + Gx1[81]*Gx2[113] + Gx1[82]*Gx2[125] + Gx1[83]*Gx2[137];
Gx3[78] = + Gx1[72]*Gx2[6] + Gx1[73]*Gx2[18] + Gx1[74]*Gx2[30] + Gx1[75]*Gx2[42] + Gx1[76]*Gx2[54] + Gx1[77]*Gx2[66] + Gx1[78]*Gx2[78] + Gx1[79]*Gx2[90] + Gx1[80]*Gx2[102] + Gx1[81]*Gx2[114] + Gx1[82]*Gx2[126] + Gx1[83]*Gx2[138];
Gx3[79] = + Gx1[72]*Gx2[7] + Gx1[73]*Gx2[19] + Gx1[74]*Gx2[31] + Gx1[75]*Gx2[43] + Gx1[76]*Gx2[55] + Gx1[77]*Gx2[67] + Gx1[78]*Gx2[79] + Gx1[79]*Gx2[91] + Gx1[80]*Gx2[103] + Gx1[81]*Gx2[115] + Gx1[82]*Gx2[127] + Gx1[83]*Gx2[139];
Gx3[80] = + Gx1[72]*Gx2[8] + Gx1[73]*Gx2[20] + Gx1[74]*Gx2[32] + Gx1[75]*Gx2[44] + Gx1[76]*Gx2[56] + Gx1[77]*Gx2[68] + Gx1[78]*Gx2[80] + Gx1[79]*Gx2[92] + Gx1[80]*Gx2[104] + Gx1[81]*Gx2[116] + Gx1[82]*Gx2[128] + Gx1[83]*Gx2[140];
Gx3[81] = + Gx1[72]*Gx2[9] + Gx1[73]*Gx2[21] + Gx1[74]*Gx2[33] + Gx1[75]*Gx2[45] + Gx1[76]*Gx2[57] + Gx1[77]*Gx2[69] + Gx1[78]*Gx2[81] + Gx1[79]*Gx2[93] + Gx1[80]*Gx2[105] + Gx1[81]*Gx2[117] + Gx1[82]*Gx2[129] + Gx1[83]*Gx2[141];
Gx3[82] = + Gx1[72]*Gx2[10] + Gx1[73]*Gx2[22] + Gx1[74]*Gx2[34] + Gx1[75]*Gx2[46] + Gx1[76]*Gx2[58] + Gx1[77]*Gx2[70] + Gx1[78]*Gx2[82] + Gx1[79]*Gx2[94] + Gx1[80]*Gx2[106] + Gx1[81]*Gx2[118] + Gx1[82]*Gx2[130] + Gx1[83]*Gx2[142];
Gx3[83] = + Gx1[72]*Gx2[11] + Gx1[73]*Gx2[23] + Gx1[74]*Gx2[35] + Gx1[75]*Gx2[47] + Gx1[76]*Gx2[59] + Gx1[77]*Gx2[71] + Gx1[78]*Gx2[83] + Gx1[79]*Gx2[95] + Gx1[80]*Gx2[107] + Gx1[81]*Gx2[119] + Gx1[82]*Gx2[131] + Gx1[83]*Gx2[143];
Gx3[84] = + Gx1[84]*Gx2[0] + Gx1[85]*Gx2[12] + Gx1[86]*Gx2[24] + Gx1[87]*Gx2[36] + Gx1[88]*Gx2[48] + Gx1[89]*Gx2[60] + Gx1[90]*Gx2[72] + Gx1[91]*Gx2[84] + Gx1[92]*Gx2[96] + Gx1[93]*Gx2[108] + Gx1[94]*Gx2[120] + Gx1[95]*Gx2[132];
Gx3[85] = + Gx1[84]*Gx2[1] + Gx1[85]*Gx2[13] + Gx1[86]*Gx2[25] + Gx1[87]*Gx2[37] + Gx1[88]*Gx2[49] + Gx1[89]*Gx2[61] + Gx1[90]*Gx2[73] + Gx1[91]*Gx2[85] + Gx1[92]*Gx2[97] + Gx1[93]*Gx2[109] + Gx1[94]*Gx2[121] + Gx1[95]*Gx2[133];
Gx3[86] = + Gx1[84]*Gx2[2] + Gx1[85]*Gx2[14] + Gx1[86]*Gx2[26] + Gx1[87]*Gx2[38] + Gx1[88]*Gx2[50] + Gx1[89]*Gx2[62] + Gx1[90]*Gx2[74] + Gx1[91]*Gx2[86] + Gx1[92]*Gx2[98] + Gx1[93]*Gx2[110] + Gx1[94]*Gx2[122] + Gx1[95]*Gx2[134];
Gx3[87] = + Gx1[84]*Gx2[3] + Gx1[85]*Gx2[15] + Gx1[86]*Gx2[27] + Gx1[87]*Gx2[39] + Gx1[88]*Gx2[51] + Gx1[89]*Gx2[63] + Gx1[90]*Gx2[75] + Gx1[91]*Gx2[87] + Gx1[92]*Gx2[99] + Gx1[93]*Gx2[111] + Gx1[94]*Gx2[123] + Gx1[95]*Gx2[135];
Gx3[88] = + Gx1[84]*Gx2[4] + Gx1[85]*Gx2[16] + Gx1[86]*Gx2[28] + Gx1[87]*Gx2[40] + Gx1[88]*Gx2[52] + Gx1[89]*Gx2[64] + Gx1[90]*Gx2[76] + Gx1[91]*Gx2[88] + Gx1[92]*Gx2[100] + Gx1[93]*Gx2[112] + Gx1[94]*Gx2[124] + Gx1[95]*Gx2[136];
Gx3[89] = + Gx1[84]*Gx2[5] + Gx1[85]*Gx2[17] + Gx1[86]*Gx2[29] + Gx1[87]*Gx2[41] + Gx1[88]*Gx2[53] + Gx1[89]*Gx2[65] + Gx1[90]*Gx2[77] + Gx1[91]*Gx2[89] + Gx1[92]*Gx2[101] + Gx1[93]*Gx2[113] + Gx1[94]*Gx2[125] + Gx1[95]*Gx2[137];
Gx3[90] = + Gx1[84]*Gx2[6] + Gx1[85]*Gx2[18] + Gx1[86]*Gx2[30] + Gx1[87]*Gx2[42] + Gx1[88]*Gx2[54] + Gx1[89]*Gx2[66] + Gx1[90]*Gx2[78] + Gx1[91]*Gx2[90] + Gx1[92]*Gx2[102] + Gx1[93]*Gx2[114] + Gx1[94]*Gx2[126] + Gx1[95]*Gx2[138];
Gx3[91] = + Gx1[84]*Gx2[7] + Gx1[85]*Gx2[19] + Gx1[86]*Gx2[31] + Gx1[87]*Gx2[43] + Gx1[88]*Gx2[55] + Gx1[89]*Gx2[67] + Gx1[90]*Gx2[79] + Gx1[91]*Gx2[91] + Gx1[92]*Gx2[103] + Gx1[93]*Gx2[115] + Gx1[94]*Gx2[127] + Gx1[95]*Gx2[139];
Gx3[92] = + Gx1[84]*Gx2[8] + Gx1[85]*Gx2[20] + Gx1[86]*Gx2[32] + Gx1[87]*Gx2[44] + Gx1[88]*Gx2[56] + Gx1[89]*Gx2[68] + Gx1[90]*Gx2[80] + Gx1[91]*Gx2[92] + Gx1[92]*Gx2[104] + Gx1[93]*Gx2[116] + Gx1[94]*Gx2[128] + Gx1[95]*Gx2[140];
Gx3[93] = + Gx1[84]*Gx2[9] + Gx1[85]*Gx2[21] + Gx1[86]*Gx2[33] + Gx1[87]*Gx2[45] + Gx1[88]*Gx2[57] + Gx1[89]*Gx2[69] + Gx1[90]*Gx2[81] + Gx1[91]*Gx2[93] + Gx1[92]*Gx2[105] + Gx1[93]*Gx2[117] + Gx1[94]*Gx2[129] + Gx1[95]*Gx2[141];
Gx3[94] = + Gx1[84]*Gx2[10] + Gx1[85]*Gx2[22] + Gx1[86]*Gx2[34] + Gx1[87]*Gx2[46] + Gx1[88]*Gx2[58] + Gx1[89]*Gx2[70] + Gx1[90]*Gx2[82] + Gx1[91]*Gx2[94] + Gx1[92]*Gx2[106] + Gx1[93]*Gx2[118] + Gx1[94]*Gx2[130] + Gx1[95]*Gx2[142];
Gx3[95] = + Gx1[84]*Gx2[11] + Gx1[85]*Gx2[23] + Gx1[86]*Gx2[35] + Gx1[87]*Gx2[47] + Gx1[88]*Gx2[59] + Gx1[89]*Gx2[71] + Gx1[90]*Gx2[83] + Gx1[91]*Gx2[95] + Gx1[92]*Gx2[107] + Gx1[93]*Gx2[119] + Gx1[94]*Gx2[131] + Gx1[95]*Gx2[143];
Gx3[96] = + Gx1[96]*Gx2[0] + Gx1[97]*Gx2[12] + Gx1[98]*Gx2[24] + Gx1[99]*Gx2[36] + Gx1[100]*Gx2[48] + Gx1[101]*Gx2[60] + Gx1[102]*Gx2[72] + Gx1[103]*Gx2[84] + Gx1[104]*Gx2[96] + Gx1[105]*Gx2[108] + Gx1[106]*Gx2[120] + Gx1[107]*Gx2[132];
Gx3[97] = + Gx1[96]*Gx2[1] + Gx1[97]*Gx2[13] + Gx1[98]*Gx2[25] + Gx1[99]*Gx2[37] + Gx1[100]*Gx2[49] + Gx1[101]*Gx2[61] + Gx1[102]*Gx2[73] + Gx1[103]*Gx2[85] + Gx1[104]*Gx2[97] + Gx1[105]*Gx2[109] + Gx1[106]*Gx2[121] + Gx1[107]*Gx2[133];
Gx3[98] = + Gx1[96]*Gx2[2] + Gx1[97]*Gx2[14] + Gx1[98]*Gx2[26] + Gx1[99]*Gx2[38] + Gx1[100]*Gx2[50] + Gx1[101]*Gx2[62] + Gx1[102]*Gx2[74] + Gx1[103]*Gx2[86] + Gx1[104]*Gx2[98] + Gx1[105]*Gx2[110] + Gx1[106]*Gx2[122] + Gx1[107]*Gx2[134];
Gx3[99] = + Gx1[96]*Gx2[3] + Gx1[97]*Gx2[15] + Gx1[98]*Gx2[27] + Gx1[99]*Gx2[39] + Gx1[100]*Gx2[51] + Gx1[101]*Gx2[63] + Gx1[102]*Gx2[75] + Gx1[103]*Gx2[87] + Gx1[104]*Gx2[99] + Gx1[105]*Gx2[111] + Gx1[106]*Gx2[123] + Gx1[107]*Gx2[135];
Gx3[100] = + Gx1[96]*Gx2[4] + Gx1[97]*Gx2[16] + Gx1[98]*Gx2[28] + Gx1[99]*Gx2[40] + Gx1[100]*Gx2[52] + Gx1[101]*Gx2[64] + Gx1[102]*Gx2[76] + Gx1[103]*Gx2[88] + Gx1[104]*Gx2[100] + Gx1[105]*Gx2[112] + Gx1[106]*Gx2[124] + Gx1[107]*Gx2[136];
Gx3[101] = + Gx1[96]*Gx2[5] + Gx1[97]*Gx2[17] + Gx1[98]*Gx2[29] + Gx1[99]*Gx2[41] + Gx1[100]*Gx2[53] + Gx1[101]*Gx2[65] + Gx1[102]*Gx2[77] + Gx1[103]*Gx2[89] + Gx1[104]*Gx2[101] + Gx1[105]*Gx2[113] + Gx1[106]*Gx2[125] + Gx1[107]*Gx2[137];
Gx3[102] = + Gx1[96]*Gx2[6] + Gx1[97]*Gx2[18] + Gx1[98]*Gx2[30] + Gx1[99]*Gx2[42] + Gx1[100]*Gx2[54] + Gx1[101]*Gx2[66] + Gx1[102]*Gx2[78] + Gx1[103]*Gx2[90] + Gx1[104]*Gx2[102] + Gx1[105]*Gx2[114] + Gx1[106]*Gx2[126] + Gx1[107]*Gx2[138];
Gx3[103] = + Gx1[96]*Gx2[7] + Gx1[97]*Gx2[19] + Gx1[98]*Gx2[31] + Gx1[99]*Gx2[43] + Gx1[100]*Gx2[55] + Gx1[101]*Gx2[67] + Gx1[102]*Gx2[79] + Gx1[103]*Gx2[91] + Gx1[104]*Gx2[103] + Gx1[105]*Gx2[115] + Gx1[106]*Gx2[127] + Gx1[107]*Gx2[139];
Gx3[104] = + Gx1[96]*Gx2[8] + Gx1[97]*Gx2[20] + Gx1[98]*Gx2[32] + Gx1[99]*Gx2[44] + Gx1[100]*Gx2[56] + Gx1[101]*Gx2[68] + Gx1[102]*Gx2[80] + Gx1[103]*Gx2[92] + Gx1[104]*Gx2[104] + Gx1[105]*Gx2[116] + Gx1[106]*Gx2[128] + Gx1[107]*Gx2[140];
Gx3[105] = + Gx1[96]*Gx2[9] + Gx1[97]*Gx2[21] + Gx1[98]*Gx2[33] + Gx1[99]*Gx2[45] + Gx1[100]*Gx2[57] + Gx1[101]*Gx2[69] + Gx1[102]*Gx2[81] + Gx1[103]*Gx2[93] + Gx1[104]*Gx2[105] + Gx1[105]*Gx2[117] + Gx1[106]*Gx2[129] + Gx1[107]*Gx2[141];
Gx3[106] = + Gx1[96]*Gx2[10] + Gx1[97]*Gx2[22] + Gx1[98]*Gx2[34] + Gx1[99]*Gx2[46] + Gx1[100]*Gx2[58] + Gx1[101]*Gx2[70] + Gx1[102]*Gx2[82] + Gx1[103]*Gx2[94] + Gx1[104]*Gx2[106] + Gx1[105]*Gx2[118] + Gx1[106]*Gx2[130] + Gx1[107]*Gx2[142];
Gx3[107] = + Gx1[96]*Gx2[11] + Gx1[97]*Gx2[23] + Gx1[98]*Gx2[35] + Gx1[99]*Gx2[47] + Gx1[100]*Gx2[59] + Gx1[101]*Gx2[71] + Gx1[102]*Gx2[83] + Gx1[103]*Gx2[95] + Gx1[104]*Gx2[107] + Gx1[105]*Gx2[119] + Gx1[106]*Gx2[131] + Gx1[107]*Gx2[143];
Gx3[108] = + Gx1[108]*Gx2[0] + Gx1[109]*Gx2[12] + Gx1[110]*Gx2[24] + Gx1[111]*Gx2[36] + Gx1[112]*Gx2[48] + Gx1[113]*Gx2[60] + Gx1[114]*Gx2[72] + Gx1[115]*Gx2[84] + Gx1[116]*Gx2[96] + Gx1[117]*Gx2[108] + Gx1[118]*Gx2[120] + Gx1[119]*Gx2[132];
Gx3[109] = + Gx1[108]*Gx2[1] + Gx1[109]*Gx2[13] + Gx1[110]*Gx2[25] + Gx1[111]*Gx2[37] + Gx1[112]*Gx2[49] + Gx1[113]*Gx2[61] + Gx1[114]*Gx2[73] + Gx1[115]*Gx2[85] + Gx1[116]*Gx2[97] + Gx1[117]*Gx2[109] + Gx1[118]*Gx2[121] + Gx1[119]*Gx2[133];
Gx3[110] = + Gx1[108]*Gx2[2] + Gx1[109]*Gx2[14] + Gx1[110]*Gx2[26] + Gx1[111]*Gx2[38] + Gx1[112]*Gx2[50] + Gx1[113]*Gx2[62] + Gx1[114]*Gx2[74] + Gx1[115]*Gx2[86] + Gx1[116]*Gx2[98] + Gx1[117]*Gx2[110] + Gx1[118]*Gx2[122] + Gx1[119]*Gx2[134];
Gx3[111] = + Gx1[108]*Gx2[3] + Gx1[109]*Gx2[15] + Gx1[110]*Gx2[27] + Gx1[111]*Gx2[39] + Gx1[112]*Gx2[51] + Gx1[113]*Gx2[63] + Gx1[114]*Gx2[75] + Gx1[115]*Gx2[87] + Gx1[116]*Gx2[99] + Gx1[117]*Gx2[111] + Gx1[118]*Gx2[123] + Gx1[119]*Gx2[135];
Gx3[112] = + Gx1[108]*Gx2[4] + Gx1[109]*Gx2[16] + Gx1[110]*Gx2[28] + Gx1[111]*Gx2[40] + Gx1[112]*Gx2[52] + Gx1[113]*Gx2[64] + Gx1[114]*Gx2[76] + Gx1[115]*Gx2[88] + Gx1[116]*Gx2[100] + Gx1[117]*Gx2[112] + Gx1[118]*Gx2[124] + Gx1[119]*Gx2[136];
Gx3[113] = + Gx1[108]*Gx2[5] + Gx1[109]*Gx2[17] + Gx1[110]*Gx2[29] + Gx1[111]*Gx2[41] + Gx1[112]*Gx2[53] + Gx1[113]*Gx2[65] + Gx1[114]*Gx2[77] + Gx1[115]*Gx2[89] + Gx1[116]*Gx2[101] + Gx1[117]*Gx2[113] + Gx1[118]*Gx2[125] + Gx1[119]*Gx2[137];
Gx3[114] = + Gx1[108]*Gx2[6] + Gx1[109]*Gx2[18] + Gx1[110]*Gx2[30] + Gx1[111]*Gx2[42] + Gx1[112]*Gx2[54] + Gx1[113]*Gx2[66] + Gx1[114]*Gx2[78] + Gx1[115]*Gx2[90] + Gx1[116]*Gx2[102] + Gx1[117]*Gx2[114] + Gx1[118]*Gx2[126] + Gx1[119]*Gx2[138];
Gx3[115] = + Gx1[108]*Gx2[7] + Gx1[109]*Gx2[19] + Gx1[110]*Gx2[31] + Gx1[111]*Gx2[43] + Gx1[112]*Gx2[55] + Gx1[113]*Gx2[67] + Gx1[114]*Gx2[79] + Gx1[115]*Gx2[91] + Gx1[116]*Gx2[103] + Gx1[117]*Gx2[115] + Gx1[118]*Gx2[127] + Gx1[119]*Gx2[139];
Gx3[116] = + Gx1[108]*Gx2[8] + Gx1[109]*Gx2[20] + Gx1[110]*Gx2[32] + Gx1[111]*Gx2[44] + Gx1[112]*Gx2[56] + Gx1[113]*Gx2[68] + Gx1[114]*Gx2[80] + Gx1[115]*Gx2[92] + Gx1[116]*Gx2[104] + Gx1[117]*Gx2[116] + Gx1[118]*Gx2[128] + Gx1[119]*Gx2[140];
Gx3[117] = + Gx1[108]*Gx2[9] + Gx1[109]*Gx2[21] + Gx1[110]*Gx2[33] + Gx1[111]*Gx2[45] + Gx1[112]*Gx2[57] + Gx1[113]*Gx2[69] + Gx1[114]*Gx2[81] + Gx1[115]*Gx2[93] + Gx1[116]*Gx2[105] + Gx1[117]*Gx2[117] + Gx1[118]*Gx2[129] + Gx1[119]*Gx2[141];
Gx3[118] = + Gx1[108]*Gx2[10] + Gx1[109]*Gx2[22] + Gx1[110]*Gx2[34] + Gx1[111]*Gx2[46] + Gx1[112]*Gx2[58] + Gx1[113]*Gx2[70] + Gx1[114]*Gx2[82] + Gx1[115]*Gx2[94] + Gx1[116]*Gx2[106] + Gx1[117]*Gx2[118] + Gx1[118]*Gx2[130] + Gx1[119]*Gx2[142];
Gx3[119] = + Gx1[108]*Gx2[11] + Gx1[109]*Gx2[23] + Gx1[110]*Gx2[35] + Gx1[111]*Gx2[47] + Gx1[112]*Gx2[59] + Gx1[113]*Gx2[71] + Gx1[114]*Gx2[83] + Gx1[115]*Gx2[95] + Gx1[116]*Gx2[107] + Gx1[117]*Gx2[119] + Gx1[118]*Gx2[131] + Gx1[119]*Gx2[143];
Gx3[120] = + Gx1[120]*Gx2[0] + Gx1[121]*Gx2[12] + Gx1[122]*Gx2[24] + Gx1[123]*Gx2[36] + Gx1[124]*Gx2[48] + Gx1[125]*Gx2[60] + Gx1[126]*Gx2[72] + Gx1[127]*Gx2[84] + Gx1[128]*Gx2[96] + Gx1[129]*Gx2[108] + Gx1[130]*Gx2[120] + Gx1[131]*Gx2[132];
Gx3[121] = + Gx1[120]*Gx2[1] + Gx1[121]*Gx2[13] + Gx1[122]*Gx2[25] + Gx1[123]*Gx2[37] + Gx1[124]*Gx2[49] + Gx1[125]*Gx2[61] + Gx1[126]*Gx2[73] + Gx1[127]*Gx2[85] + Gx1[128]*Gx2[97] + Gx1[129]*Gx2[109] + Gx1[130]*Gx2[121] + Gx1[131]*Gx2[133];
Gx3[122] = + Gx1[120]*Gx2[2] + Gx1[121]*Gx2[14] + Gx1[122]*Gx2[26] + Gx1[123]*Gx2[38] + Gx1[124]*Gx2[50] + Gx1[125]*Gx2[62] + Gx1[126]*Gx2[74] + Gx1[127]*Gx2[86] + Gx1[128]*Gx2[98] + Gx1[129]*Gx2[110] + Gx1[130]*Gx2[122] + Gx1[131]*Gx2[134];
Gx3[123] = + Gx1[120]*Gx2[3] + Gx1[121]*Gx2[15] + Gx1[122]*Gx2[27] + Gx1[123]*Gx2[39] + Gx1[124]*Gx2[51] + Gx1[125]*Gx2[63] + Gx1[126]*Gx2[75] + Gx1[127]*Gx2[87] + Gx1[128]*Gx2[99] + Gx1[129]*Gx2[111] + Gx1[130]*Gx2[123] + Gx1[131]*Gx2[135];
Gx3[124] = + Gx1[120]*Gx2[4] + Gx1[121]*Gx2[16] + Gx1[122]*Gx2[28] + Gx1[123]*Gx2[40] + Gx1[124]*Gx2[52] + Gx1[125]*Gx2[64] + Gx1[126]*Gx2[76] + Gx1[127]*Gx2[88] + Gx1[128]*Gx2[100] + Gx1[129]*Gx2[112] + Gx1[130]*Gx2[124] + Gx1[131]*Gx2[136];
Gx3[125] = + Gx1[120]*Gx2[5] + Gx1[121]*Gx2[17] + Gx1[122]*Gx2[29] + Gx1[123]*Gx2[41] + Gx1[124]*Gx2[53] + Gx1[125]*Gx2[65] + Gx1[126]*Gx2[77] + Gx1[127]*Gx2[89] + Gx1[128]*Gx2[101] + Gx1[129]*Gx2[113] + Gx1[130]*Gx2[125] + Gx1[131]*Gx2[137];
Gx3[126] = + Gx1[120]*Gx2[6] + Gx1[121]*Gx2[18] + Gx1[122]*Gx2[30] + Gx1[123]*Gx2[42] + Gx1[124]*Gx2[54] + Gx1[125]*Gx2[66] + Gx1[126]*Gx2[78] + Gx1[127]*Gx2[90] + Gx1[128]*Gx2[102] + Gx1[129]*Gx2[114] + Gx1[130]*Gx2[126] + Gx1[131]*Gx2[138];
Gx3[127] = + Gx1[120]*Gx2[7] + Gx1[121]*Gx2[19] + Gx1[122]*Gx2[31] + Gx1[123]*Gx2[43] + Gx1[124]*Gx2[55] + Gx1[125]*Gx2[67] + Gx1[126]*Gx2[79] + Gx1[127]*Gx2[91] + Gx1[128]*Gx2[103] + Gx1[129]*Gx2[115] + Gx1[130]*Gx2[127] + Gx1[131]*Gx2[139];
Gx3[128] = + Gx1[120]*Gx2[8] + Gx1[121]*Gx2[20] + Gx1[122]*Gx2[32] + Gx1[123]*Gx2[44] + Gx1[124]*Gx2[56] + Gx1[125]*Gx2[68] + Gx1[126]*Gx2[80] + Gx1[127]*Gx2[92] + Gx1[128]*Gx2[104] + Gx1[129]*Gx2[116] + Gx1[130]*Gx2[128] + Gx1[131]*Gx2[140];
Gx3[129] = + Gx1[120]*Gx2[9] + Gx1[121]*Gx2[21] + Gx1[122]*Gx2[33] + Gx1[123]*Gx2[45] + Gx1[124]*Gx2[57] + Gx1[125]*Gx2[69] + Gx1[126]*Gx2[81] + Gx1[127]*Gx2[93] + Gx1[128]*Gx2[105] + Gx1[129]*Gx2[117] + Gx1[130]*Gx2[129] + Gx1[131]*Gx2[141];
Gx3[130] = + Gx1[120]*Gx2[10] + Gx1[121]*Gx2[22] + Gx1[122]*Gx2[34] + Gx1[123]*Gx2[46] + Gx1[124]*Gx2[58] + Gx1[125]*Gx2[70] + Gx1[126]*Gx2[82] + Gx1[127]*Gx2[94] + Gx1[128]*Gx2[106] + Gx1[129]*Gx2[118] + Gx1[130]*Gx2[130] + Gx1[131]*Gx2[142];
Gx3[131] = + Gx1[120]*Gx2[11] + Gx1[121]*Gx2[23] + Gx1[122]*Gx2[35] + Gx1[123]*Gx2[47] + Gx1[124]*Gx2[59] + Gx1[125]*Gx2[71] + Gx1[126]*Gx2[83] + Gx1[127]*Gx2[95] + Gx1[128]*Gx2[107] + Gx1[129]*Gx2[119] + Gx1[130]*Gx2[131] + Gx1[131]*Gx2[143];
Gx3[132] = + Gx1[132]*Gx2[0] + Gx1[133]*Gx2[12] + Gx1[134]*Gx2[24] + Gx1[135]*Gx2[36] + Gx1[136]*Gx2[48] + Gx1[137]*Gx2[60] + Gx1[138]*Gx2[72] + Gx1[139]*Gx2[84] + Gx1[140]*Gx2[96] + Gx1[141]*Gx2[108] + Gx1[142]*Gx2[120] + Gx1[143]*Gx2[132];
Gx3[133] = + Gx1[132]*Gx2[1] + Gx1[133]*Gx2[13] + Gx1[134]*Gx2[25] + Gx1[135]*Gx2[37] + Gx1[136]*Gx2[49] + Gx1[137]*Gx2[61] + Gx1[138]*Gx2[73] + Gx1[139]*Gx2[85] + Gx1[140]*Gx2[97] + Gx1[141]*Gx2[109] + Gx1[142]*Gx2[121] + Gx1[143]*Gx2[133];
Gx3[134] = + Gx1[132]*Gx2[2] + Gx1[133]*Gx2[14] + Gx1[134]*Gx2[26] + Gx1[135]*Gx2[38] + Gx1[136]*Gx2[50] + Gx1[137]*Gx2[62] + Gx1[138]*Gx2[74] + Gx1[139]*Gx2[86] + Gx1[140]*Gx2[98] + Gx1[141]*Gx2[110] + Gx1[142]*Gx2[122] + Gx1[143]*Gx2[134];
Gx3[135] = + Gx1[132]*Gx2[3] + Gx1[133]*Gx2[15] + Gx1[134]*Gx2[27] + Gx1[135]*Gx2[39] + Gx1[136]*Gx2[51] + Gx1[137]*Gx2[63] + Gx1[138]*Gx2[75] + Gx1[139]*Gx2[87] + Gx1[140]*Gx2[99] + Gx1[141]*Gx2[111] + Gx1[142]*Gx2[123] + Gx1[143]*Gx2[135];
Gx3[136] = + Gx1[132]*Gx2[4] + Gx1[133]*Gx2[16] + Gx1[134]*Gx2[28] + Gx1[135]*Gx2[40] + Gx1[136]*Gx2[52] + Gx1[137]*Gx2[64] + Gx1[138]*Gx2[76] + Gx1[139]*Gx2[88] + Gx1[140]*Gx2[100] + Gx1[141]*Gx2[112] + Gx1[142]*Gx2[124] + Gx1[143]*Gx2[136];
Gx3[137] = + Gx1[132]*Gx2[5] + Gx1[133]*Gx2[17] + Gx1[134]*Gx2[29] + Gx1[135]*Gx2[41] + Gx1[136]*Gx2[53] + Gx1[137]*Gx2[65] + Gx1[138]*Gx2[77] + Gx1[139]*Gx2[89] + Gx1[140]*Gx2[101] + Gx1[141]*Gx2[113] + Gx1[142]*Gx2[125] + Gx1[143]*Gx2[137];
Gx3[138] = + Gx1[132]*Gx2[6] + Gx1[133]*Gx2[18] + Gx1[134]*Gx2[30] + Gx1[135]*Gx2[42] + Gx1[136]*Gx2[54] + Gx1[137]*Gx2[66] + Gx1[138]*Gx2[78] + Gx1[139]*Gx2[90] + Gx1[140]*Gx2[102] + Gx1[141]*Gx2[114] + Gx1[142]*Gx2[126] + Gx1[143]*Gx2[138];
Gx3[139] = + Gx1[132]*Gx2[7] + Gx1[133]*Gx2[19] + Gx1[134]*Gx2[31] + Gx1[135]*Gx2[43] + Gx1[136]*Gx2[55] + Gx1[137]*Gx2[67] + Gx1[138]*Gx2[79] + Gx1[139]*Gx2[91] + Gx1[140]*Gx2[103] + Gx1[141]*Gx2[115] + Gx1[142]*Gx2[127] + Gx1[143]*Gx2[139];
Gx3[140] = + Gx1[132]*Gx2[8] + Gx1[133]*Gx2[20] + Gx1[134]*Gx2[32] + Gx1[135]*Gx2[44] + Gx1[136]*Gx2[56] + Gx1[137]*Gx2[68] + Gx1[138]*Gx2[80] + Gx1[139]*Gx2[92] + Gx1[140]*Gx2[104] + Gx1[141]*Gx2[116] + Gx1[142]*Gx2[128] + Gx1[143]*Gx2[140];
Gx3[141] = + Gx1[132]*Gx2[9] + Gx1[133]*Gx2[21] + Gx1[134]*Gx2[33] + Gx1[135]*Gx2[45] + Gx1[136]*Gx2[57] + Gx1[137]*Gx2[69] + Gx1[138]*Gx2[81] + Gx1[139]*Gx2[93] + Gx1[140]*Gx2[105] + Gx1[141]*Gx2[117] + Gx1[142]*Gx2[129] + Gx1[143]*Gx2[141];
Gx3[142] = + Gx1[132]*Gx2[10] + Gx1[133]*Gx2[22] + Gx1[134]*Gx2[34] + Gx1[135]*Gx2[46] + Gx1[136]*Gx2[58] + Gx1[137]*Gx2[70] + Gx1[138]*Gx2[82] + Gx1[139]*Gx2[94] + Gx1[140]*Gx2[106] + Gx1[141]*Gx2[118] + Gx1[142]*Gx2[130] + Gx1[143]*Gx2[142];
Gx3[143] = + Gx1[132]*Gx2[11] + Gx1[133]*Gx2[23] + Gx1[134]*Gx2[35] + Gx1[135]*Gx2[47] + Gx1[136]*Gx2[59] + Gx1[137]*Gx2[71] + Gx1[138]*Gx2[83] + Gx1[139]*Gx2[95] + Gx1[140]*Gx2[107] + Gx1[141]*Gx2[119] + Gx1[142]*Gx2[131] + Gx1[143]*Gx2[143];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32] + Gx1[9]*Gu1[36] + Gx1[10]*Gu1[40] + Gx1[11]*Gu1[44];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33] + Gx1[9]*Gu1[37] + Gx1[10]*Gu1[41] + Gx1[11]*Gu1[45];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34] + Gx1[9]*Gu1[38] + Gx1[10]*Gu1[42] + Gx1[11]*Gu1[46];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35] + Gx1[9]*Gu1[39] + Gx1[10]*Gu1[43] + Gx1[11]*Gu1[47];
Gu2[4] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[14]*Gu1[8] + Gx1[15]*Gu1[12] + Gx1[16]*Gu1[16] + Gx1[17]*Gu1[20] + Gx1[18]*Gu1[24] + Gx1[19]*Gu1[28] + Gx1[20]*Gu1[32] + Gx1[21]*Gu1[36] + Gx1[22]*Gu1[40] + Gx1[23]*Gu1[44];
Gu2[5] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[14]*Gu1[9] + Gx1[15]*Gu1[13] + Gx1[16]*Gu1[17] + Gx1[17]*Gu1[21] + Gx1[18]*Gu1[25] + Gx1[19]*Gu1[29] + Gx1[20]*Gu1[33] + Gx1[21]*Gu1[37] + Gx1[22]*Gu1[41] + Gx1[23]*Gu1[45];
Gu2[6] = + Gx1[12]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[10] + Gx1[15]*Gu1[14] + Gx1[16]*Gu1[18] + Gx1[17]*Gu1[22] + Gx1[18]*Gu1[26] + Gx1[19]*Gu1[30] + Gx1[20]*Gu1[34] + Gx1[21]*Gu1[38] + Gx1[22]*Gu1[42] + Gx1[23]*Gu1[46];
Gu2[7] = + Gx1[12]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[11] + Gx1[15]*Gu1[15] + Gx1[16]*Gu1[19] + Gx1[17]*Gu1[23] + Gx1[18]*Gu1[27] + Gx1[19]*Gu1[31] + Gx1[20]*Gu1[35] + Gx1[21]*Gu1[39] + Gx1[22]*Gu1[43] + Gx1[23]*Gu1[47];
Gu2[8] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[27]*Gu1[12] + Gx1[28]*Gu1[16] + Gx1[29]*Gu1[20] + Gx1[30]*Gu1[24] + Gx1[31]*Gu1[28] + Gx1[32]*Gu1[32] + Gx1[33]*Gu1[36] + Gx1[34]*Gu1[40] + Gx1[35]*Gu1[44];
Gu2[9] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[27]*Gu1[13] + Gx1[28]*Gu1[17] + Gx1[29]*Gu1[21] + Gx1[30]*Gu1[25] + Gx1[31]*Gu1[29] + Gx1[32]*Gu1[33] + Gx1[33]*Gu1[37] + Gx1[34]*Gu1[41] + Gx1[35]*Gu1[45];
Gu2[10] = + Gx1[24]*Gu1[2] + Gx1[25]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[27]*Gu1[14] + Gx1[28]*Gu1[18] + Gx1[29]*Gu1[22] + Gx1[30]*Gu1[26] + Gx1[31]*Gu1[30] + Gx1[32]*Gu1[34] + Gx1[33]*Gu1[38] + Gx1[34]*Gu1[42] + Gx1[35]*Gu1[46];
Gu2[11] = + Gx1[24]*Gu1[3] + Gx1[25]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[27]*Gu1[15] + Gx1[28]*Gu1[19] + Gx1[29]*Gu1[23] + Gx1[30]*Gu1[27] + Gx1[31]*Gu1[31] + Gx1[32]*Gu1[35] + Gx1[33]*Gu1[39] + Gx1[34]*Gu1[43] + Gx1[35]*Gu1[47];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[40]*Gu1[16] + Gx1[41]*Gu1[20] + Gx1[42]*Gu1[24] + Gx1[43]*Gu1[28] + Gx1[44]*Gu1[32] + Gx1[45]*Gu1[36] + Gx1[46]*Gu1[40] + Gx1[47]*Gu1[44];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[40]*Gu1[17] + Gx1[41]*Gu1[21] + Gx1[42]*Gu1[25] + Gx1[43]*Gu1[29] + Gx1[44]*Gu1[33] + Gx1[45]*Gu1[37] + Gx1[46]*Gu1[41] + Gx1[47]*Gu1[45];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[6] + Gx1[38]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[40]*Gu1[18] + Gx1[41]*Gu1[22] + Gx1[42]*Gu1[26] + Gx1[43]*Gu1[30] + Gx1[44]*Gu1[34] + Gx1[45]*Gu1[38] + Gx1[46]*Gu1[42] + Gx1[47]*Gu1[46];
Gu2[15] = + Gx1[36]*Gu1[3] + Gx1[37]*Gu1[7] + Gx1[38]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[40]*Gu1[19] + Gx1[41]*Gu1[23] + Gx1[42]*Gu1[27] + Gx1[43]*Gu1[31] + Gx1[44]*Gu1[35] + Gx1[45]*Gu1[39] + Gx1[46]*Gu1[43] + Gx1[47]*Gu1[47];
Gu2[16] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[4] + Gx1[50]*Gu1[8] + Gx1[51]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[54]*Gu1[24] + Gx1[55]*Gu1[28] + Gx1[56]*Gu1[32] + Gx1[57]*Gu1[36] + Gx1[58]*Gu1[40] + Gx1[59]*Gu1[44];
Gu2[17] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[5] + Gx1[50]*Gu1[9] + Gx1[51]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[54]*Gu1[25] + Gx1[55]*Gu1[29] + Gx1[56]*Gu1[33] + Gx1[57]*Gu1[37] + Gx1[58]*Gu1[41] + Gx1[59]*Gu1[45];
Gu2[18] = + Gx1[48]*Gu1[2] + Gx1[49]*Gu1[6] + Gx1[50]*Gu1[10] + Gx1[51]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[54]*Gu1[26] + Gx1[55]*Gu1[30] + Gx1[56]*Gu1[34] + Gx1[57]*Gu1[38] + Gx1[58]*Gu1[42] + Gx1[59]*Gu1[46];
Gu2[19] = + Gx1[48]*Gu1[3] + Gx1[49]*Gu1[7] + Gx1[50]*Gu1[11] + Gx1[51]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[54]*Gu1[27] + Gx1[55]*Gu1[31] + Gx1[56]*Gu1[35] + Gx1[57]*Gu1[39] + Gx1[58]*Gu1[43] + Gx1[59]*Gu1[47];
Gu2[20] = + Gx1[60]*Gu1[0] + Gx1[61]*Gu1[4] + Gx1[62]*Gu1[8] + Gx1[63]*Gu1[12] + Gx1[64]*Gu1[16] + Gx1[65]*Gu1[20] + Gx1[66]*Gu1[24] + Gx1[67]*Gu1[28] + Gx1[68]*Gu1[32] + Gx1[69]*Gu1[36] + Gx1[70]*Gu1[40] + Gx1[71]*Gu1[44];
Gu2[21] = + Gx1[60]*Gu1[1] + Gx1[61]*Gu1[5] + Gx1[62]*Gu1[9] + Gx1[63]*Gu1[13] + Gx1[64]*Gu1[17] + Gx1[65]*Gu1[21] + Gx1[66]*Gu1[25] + Gx1[67]*Gu1[29] + Gx1[68]*Gu1[33] + Gx1[69]*Gu1[37] + Gx1[70]*Gu1[41] + Gx1[71]*Gu1[45];
Gu2[22] = + Gx1[60]*Gu1[2] + Gx1[61]*Gu1[6] + Gx1[62]*Gu1[10] + Gx1[63]*Gu1[14] + Gx1[64]*Gu1[18] + Gx1[65]*Gu1[22] + Gx1[66]*Gu1[26] + Gx1[67]*Gu1[30] + Gx1[68]*Gu1[34] + Gx1[69]*Gu1[38] + Gx1[70]*Gu1[42] + Gx1[71]*Gu1[46];
Gu2[23] = + Gx1[60]*Gu1[3] + Gx1[61]*Gu1[7] + Gx1[62]*Gu1[11] + Gx1[63]*Gu1[15] + Gx1[64]*Gu1[19] + Gx1[65]*Gu1[23] + Gx1[66]*Gu1[27] + Gx1[67]*Gu1[31] + Gx1[68]*Gu1[35] + Gx1[69]*Gu1[39] + Gx1[70]*Gu1[43] + Gx1[71]*Gu1[47];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[12] + Gx1[76]*Gu1[16] + Gx1[77]*Gu1[20] + Gx1[78]*Gu1[24] + Gx1[79]*Gu1[28] + Gx1[80]*Gu1[32] + Gx1[81]*Gu1[36] + Gx1[82]*Gu1[40] + Gx1[83]*Gu1[44];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[9] + Gx1[75]*Gu1[13] + Gx1[76]*Gu1[17] + Gx1[77]*Gu1[21] + Gx1[78]*Gu1[25] + Gx1[79]*Gu1[29] + Gx1[80]*Gu1[33] + Gx1[81]*Gu1[37] + Gx1[82]*Gu1[41] + Gx1[83]*Gu1[45];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[6] + Gx1[74]*Gu1[10] + Gx1[75]*Gu1[14] + Gx1[76]*Gu1[18] + Gx1[77]*Gu1[22] + Gx1[78]*Gu1[26] + Gx1[79]*Gu1[30] + Gx1[80]*Gu1[34] + Gx1[81]*Gu1[38] + Gx1[82]*Gu1[42] + Gx1[83]*Gu1[46];
Gu2[27] = + Gx1[72]*Gu1[3] + Gx1[73]*Gu1[7] + Gx1[74]*Gu1[11] + Gx1[75]*Gu1[15] + Gx1[76]*Gu1[19] + Gx1[77]*Gu1[23] + Gx1[78]*Gu1[27] + Gx1[79]*Gu1[31] + Gx1[80]*Gu1[35] + Gx1[81]*Gu1[39] + Gx1[82]*Gu1[43] + Gx1[83]*Gu1[47];
Gu2[28] = + Gx1[84]*Gu1[0] + Gx1[85]*Gu1[4] + Gx1[86]*Gu1[8] + Gx1[87]*Gu1[12] + Gx1[88]*Gu1[16] + Gx1[89]*Gu1[20] + Gx1[90]*Gu1[24] + Gx1[91]*Gu1[28] + Gx1[92]*Gu1[32] + Gx1[93]*Gu1[36] + Gx1[94]*Gu1[40] + Gx1[95]*Gu1[44];
Gu2[29] = + Gx1[84]*Gu1[1] + Gx1[85]*Gu1[5] + Gx1[86]*Gu1[9] + Gx1[87]*Gu1[13] + Gx1[88]*Gu1[17] + Gx1[89]*Gu1[21] + Gx1[90]*Gu1[25] + Gx1[91]*Gu1[29] + Gx1[92]*Gu1[33] + Gx1[93]*Gu1[37] + Gx1[94]*Gu1[41] + Gx1[95]*Gu1[45];
Gu2[30] = + Gx1[84]*Gu1[2] + Gx1[85]*Gu1[6] + Gx1[86]*Gu1[10] + Gx1[87]*Gu1[14] + Gx1[88]*Gu1[18] + Gx1[89]*Gu1[22] + Gx1[90]*Gu1[26] + Gx1[91]*Gu1[30] + Gx1[92]*Gu1[34] + Gx1[93]*Gu1[38] + Gx1[94]*Gu1[42] + Gx1[95]*Gu1[46];
Gu2[31] = + Gx1[84]*Gu1[3] + Gx1[85]*Gu1[7] + Gx1[86]*Gu1[11] + Gx1[87]*Gu1[15] + Gx1[88]*Gu1[19] + Gx1[89]*Gu1[23] + Gx1[90]*Gu1[27] + Gx1[91]*Gu1[31] + Gx1[92]*Gu1[35] + Gx1[93]*Gu1[39] + Gx1[94]*Gu1[43] + Gx1[95]*Gu1[47];
Gu2[32] = + Gx1[96]*Gu1[0] + Gx1[97]*Gu1[4] + Gx1[98]*Gu1[8] + Gx1[99]*Gu1[12] + Gx1[100]*Gu1[16] + Gx1[101]*Gu1[20] + Gx1[102]*Gu1[24] + Gx1[103]*Gu1[28] + Gx1[104]*Gu1[32] + Gx1[105]*Gu1[36] + Gx1[106]*Gu1[40] + Gx1[107]*Gu1[44];
Gu2[33] = + Gx1[96]*Gu1[1] + Gx1[97]*Gu1[5] + Gx1[98]*Gu1[9] + Gx1[99]*Gu1[13] + Gx1[100]*Gu1[17] + Gx1[101]*Gu1[21] + Gx1[102]*Gu1[25] + Gx1[103]*Gu1[29] + Gx1[104]*Gu1[33] + Gx1[105]*Gu1[37] + Gx1[106]*Gu1[41] + Gx1[107]*Gu1[45];
Gu2[34] = + Gx1[96]*Gu1[2] + Gx1[97]*Gu1[6] + Gx1[98]*Gu1[10] + Gx1[99]*Gu1[14] + Gx1[100]*Gu1[18] + Gx1[101]*Gu1[22] + Gx1[102]*Gu1[26] + Gx1[103]*Gu1[30] + Gx1[104]*Gu1[34] + Gx1[105]*Gu1[38] + Gx1[106]*Gu1[42] + Gx1[107]*Gu1[46];
Gu2[35] = + Gx1[96]*Gu1[3] + Gx1[97]*Gu1[7] + Gx1[98]*Gu1[11] + Gx1[99]*Gu1[15] + Gx1[100]*Gu1[19] + Gx1[101]*Gu1[23] + Gx1[102]*Gu1[27] + Gx1[103]*Gu1[31] + Gx1[104]*Gu1[35] + Gx1[105]*Gu1[39] + Gx1[106]*Gu1[43] + Gx1[107]*Gu1[47];
Gu2[36] = + Gx1[108]*Gu1[0] + Gx1[109]*Gu1[4] + Gx1[110]*Gu1[8] + Gx1[111]*Gu1[12] + Gx1[112]*Gu1[16] + Gx1[113]*Gu1[20] + Gx1[114]*Gu1[24] + Gx1[115]*Gu1[28] + Gx1[116]*Gu1[32] + Gx1[117]*Gu1[36] + Gx1[118]*Gu1[40] + Gx1[119]*Gu1[44];
Gu2[37] = + Gx1[108]*Gu1[1] + Gx1[109]*Gu1[5] + Gx1[110]*Gu1[9] + Gx1[111]*Gu1[13] + Gx1[112]*Gu1[17] + Gx1[113]*Gu1[21] + Gx1[114]*Gu1[25] + Gx1[115]*Gu1[29] + Gx1[116]*Gu1[33] + Gx1[117]*Gu1[37] + Gx1[118]*Gu1[41] + Gx1[119]*Gu1[45];
Gu2[38] = + Gx1[108]*Gu1[2] + Gx1[109]*Gu1[6] + Gx1[110]*Gu1[10] + Gx1[111]*Gu1[14] + Gx1[112]*Gu1[18] + Gx1[113]*Gu1[22] + Gx1[114]*Gu1[26] + Gx1[115]*Gu1[30] + Gx1[116]*Gu1[34] + Gx1[117]*Gu1[38] + Gx1[118]*Gu1[42] + Gx1[119]*Gu1[46];
Gu2[39] = + Gx1[108]*Gu1[3] + Gx1[109]*Gu1[7] + Gx1[110]*Gu1[11] + Gx1[111]*Gu1[15] + Gx1[112]*Gu1[19] + Gx1[113]*Gu1[23] + Gx1[114]*Gu1[27] + Gx1[115]*Gu1[31] + Gx1[116]*Gu1[35] + Gx1[117]*Gu1[39] + Gx1[118]*Gu1[43] + Gx1[119]*Gu1[47];
Gu2[40] = + Gx1[120]*Gu1[0] + Gx1[121]*Gu1[4] + Gx1[122]*Gu1[8] + Gx1[123]*Gu1[12] + Gx1[124]*Gu1[16] + Gx1[125]*Gu1[20] + Gx1[126]*Gu1[24] + Gx1[127]*Gu1[28] + Gx1[128]*Gu1[32] + Gx1[129]*Gu1[36] + Gx1[130]*Gu1[40] + Gx1[131]*Gu1[44];
Gu2[41] = + Gx1[120]*Gu1[1] + Gx1[121]*Gu1[5] + Gx1[122]*Gu1[9] + Gx1[123]*Gu1[13] + Gx1[124]*Gu1[17] + Gx1[125]*Gu1[21] + Gx1[126]*Gu1[25] + Gx1[127]*Gu1[29] + Gx1[128]*Gu1[33] + Gx1[129]*Gu1[37] + Gx1[130]*Gu1[41] + Gx1[131]*Gu1[45];
Gu2[42] = + Gx1[120]*Gu1[2] + Gx1[121]*Gu1[6] + Gx1[122]*Gu1[10] + Gx1[123]*Gu1[14] + Gx1[124]*Gu1[18] + Gx1[125]*Gu1[22] + Gx1[126]*Gu1[26] + Gx1[127]*Gu1[30] + Gx1[128]*Gu1[34] + Gx1[129]*Gu1[38] + Gx1[130]*Gu1[42] + Gx1[131]*Gu1[46];
Gu2[43] = + Gx1[120]*Gu1[3] + Gx1[121]*Gu1[7] + Gx1[122]*Gu1[11] + Gx1[123]*Gu1[15] + Gx1[124]*Gu1[19] + Gx1[125]*Gu1[23] + Gx1[126]*Gu1[27] + Gx1[127]*Gu1[31] + Gx1[128]*Gu1[35] + Gx1[129]*Gu1[39] + Gx1[130]*Gu1[43] + Gx1[131]*Gu1[47];
Gu2[44] = + Gx1[132]*Gu1[0] + Gx1[133]*Gu1[4] + Gx1[134]*Gu1[8] + Gx1[135]*Gu1[12] + Gx1[136]*Gu1[16] + Gx1[137]*Gu1[20] + Gx1[138]*Gu1[24] + Gx1[139]*Gu1[28] + Gx1[140]*Gu1[32] + Gx1[141]*Gu1[36] + Gx1[142]*Gu1[40] + Gx1[143]*Gu1[44];
Gu2[45] = + Gx1[132]*Gu1[1] + Gx1[133]*Gu1[5] + Gx1[134]*Gu1[9] + Gx1[135]*Gu1[13] + Gx1[136]*Gu1[17] + Gx1[137]*Gu1[21] + Gx1[138]*Gu1[25] + Gx1[139]*Gu1[29] + Gx1[140]*Gu1[33] + Gx1[141]*Gu1[37] + Gx1[142]*Gu1[41] + Gx1[143]*Gu1[45];
Gu2[46] = + Gx1[132]*Gu1[2] + Gx1[133]*Gu1[6] + Gx1[134]*Gu1[10] + Gx1[135]*Gu1[14] + Gx1[136]*Gu1[18] + Gx1[137]*Gu1[22] + Gx1[138]*Gu1[26] + Gx1[139]*Gu1[30] + Gx1[140]*Gu1[34] + Gx1[141]*Gu1[38] + Gx1[142]*Gu1[42] + Gx1[143]*Gu1[46];
Gu2[47] = + Gx1[132]*Gu1[3] + Gx1[133]*Gu1[7] + Gx1[134]*Gu1[11] + Gx1[135]*Gu1[15] + Gx1[136]*Gu1[19] + Gx1[137]*Gu1[23] + Gx1[138]*Gu1[27] + Gx1[139]*Gu1[31] + Gx1[140]*Gu1[35] + Gx1[141]*Gu1[39] + Gx1[142]*Gu1[43] + Gx1[143]*Gu1[47];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
Gu2[40] = Gu1[40];
Gu2[41] = Gu1[41];
Gu2[42] = Gu1[42];
Gu2[43] = Gu1[43];
Gu2[44] = Gu1[44];
Gu2[45] = Gu1[45];
Gu2[46] = Gu1[46];
Gu2[47] = Gu1[47];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 480) + (iCol * 4)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + Gu1[40]*Gu2[40] + Gu1[44]*Gu2[44];
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 1)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + Gu1[40]*Gu2[41] + Gu1[44]*Gu2[45];
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 2)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + Gu1[40]*Gu2[42] + Gu1[44]*Gu2[46];
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 3)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + Gu1[40]*Gu2[43] + Gu1[44]*Gu2[47];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + Gu1[41]*Gu2[40] + Gu1[45]*Gu2[44];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 1)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + Gu1[41]*Gu2[41] + Gu1[45]*Gu2[45];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 2)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + Gu1[41]*Gu2[42] + Gu1[45]*Gu2[46];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 3)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + Gu1[41]*Gu2[43] + Gu1[45]*Gu2[47];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + Gu1[42]*Gu2[40] + Gu1[46]*Gu2[44];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 1)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + Gu1[42]*Gu2[41] + Gu1[46]*Gu2[45];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 2)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + Gu1[42]*Gu2[42] + Gu1[46]*Gu2[46];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 3)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + Gu1[42]*Gu2[43] + Gu1[46]*Gu2[47];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + Gu1[43]*Gu2[40] + Gu1[47]*Gu2[44];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 1)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + Gu1[43]*Gu2[41] + Gu1[47]*Gu2[45];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 2)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + Gu1[43]*Gu2[42] + Gu1[47]*Gu2[46];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 3)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + Gu1[43]*Gu2[43] + Gu1[47]*Gu2[47];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 480) + (iCol * 4)] = R11[0];
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 2)] = R11[2];
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 3)] = R11[3];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4)] = R11[4];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 1)] = R11[5];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 2)] = R11[6];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 3)] = R11[7];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4)] = R11[8];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 1)] = R11[9];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 2)] = R11[10];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 3)] = R11[11];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4)] = R11[12];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 1)] = R11[13];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 2)] = R11[14];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 3)] = R11[15];
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 480) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 3)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 480) + (iCol * 4)] = acadoWorkspace.H[(iCol * 480) + (iRow * 4)];
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 480 + 120) + (iRow * 4)];
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 480 + 240) + (iRow * 4)];
acadoWorkspace.H[(iRow * 480) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 480 + 360) + (iRow * 4)];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4)] = acadoWorkspace.H[(iCol * 480) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 480 + 120) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 480 + 240) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 480 + 120) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 480 + 360) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4)] = acadoWorkspace.H[(iCol * 480) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 480 + 120) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 480 + 240) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 480 + 240) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 480 + 360) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4)] = acadoWorkspace.H[(iCol * 480) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 480 + 120) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 480 + 240) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 480 + 360) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 480 + 360) + (iRow * 4 + 3)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8] + Gx1[9]*dOld[9] + Gx1[10]*dOld[10] + Gx1[11]*dOld[11];
dNew[1] = + Gx1[12]*dOld[0] + Gx1[13]*dOld[1] + Gx1[14]*dOld[2] + Gx1[15]*dOld[3] + Gx1[16]*dOld[4] + Gx1[17]*dOld[5] + Gx1[18]*dOld[6] + Gx1[19]*dOld[7] + Gx1[20]*dOld[8] + Gx1[21]*dOld[9] + Gx1[22]*dOld[10] + Gx1[23]*dOld[11];
dNew[2] = + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7] + Gx1[32]*dOld[8] + Gx1[33]*dOld[9] + Gx1[34]*dOld[10] + Gx1[35]*dOld[11];
dNew[3] = + Gx1[36]*dOld[0] + Gx1[37]*dOld[1] + Gx1[38]*dOld[2] + Gx1[39]*dOld[3] + Gx1[40]*dOld[4] + Gx1[41]*dOld[5] + Gx1[42]*dOld[6] + Gx1[43]*dOld[7] + Gx1[44]*dOld[8] + Gx1[45]*dOld[9] + Gx1[46]*dOld[10] + Gx1[47]*dOld[11];
dNew[4] = + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7] + Gx1[56]*dOld[8] + Gx1[57]*dOld[9] + Gx1[58]*dOld[10] + Gx1[59]*dOld[11];
dNew[5] = + Gx1[60]*dOld[0] + Gx1[61]*dOld[1] + Gx1[62]*dOld[2] + Gx1[63]*dOld[3] + Gx1[64]*dOld[4] + Gx1[65]*dOld[5] + Gx1[66]*dOld[6] + Gx1[67]*dOld[7] + Gx1[68]*dOld[8] + Gx1[69]*dOld[9] + Gx1[70]*dOld[10] + Gx1[71]*dOld[11];
dNew[6] = + Gx1[72]*dOld[0] + Gx1[73]*dOld[1] + Gx1[74]*dOld[2] + Gx1[75]*dOld[3] + Gx1[76]*dOld[4] + Gx1[77]*dOld[5] + Gx1[78]*dOld[6] + Gx1[79]*dOld[7] + Gx1[80]*dOld[8] + Gx1[81]*dOld[9] + Gx1[82]*dOld[10] + Gx1[83]*dOld[11];
dNew[7] = + Gx1[84]*dOld[0] + Gx1[85]*dOld[1] + Gx1[86]*dOld[2] + Gx1[87]*dOld[3] + Gx1[88]*dOld[4] + Gx1[89]*dOld[5] + Gx1[90]*dOld[6] + Gx1[91]*dOld[7] + Gx1[92]*dOld[8] + Gx1[93]*dOld[9] + Gx1[94]*dOld[10] + Gx1[95]*dOld[11];
dNew[8] = + Gx1[96]*dOld[0] + Gx1[97]*dOld[1] + Gx1[98]*dOld[2] + Gx1[99]*dOld[3] + Gx1[100]*dOld[4] + Gx1[101]*dOld[5] + Gx1[102]*dOld[6] + Gx1[103]*dOld[7] + Gx1[104]*dOld[8] + Gx1[105]*dOld[9] + Gx1[106]*dOld[10] + Gx1[107]*dOld[11];
dNew[9] = + Gx1[108]*dOld[0] + Gx1[109]*dOld[1] + Gx1[110]*dOld[2] + Gx1[111]*dOld[3] + Gx1[112]*dOld[4] + Gx1[113]*dOld[5] + Gx1[114]*dOld[6] + Gx1[115]*dOld[7] + Gx1[116]*dOld[8] + Gx1[117]*dOld[9] + Gx1[118]*dOld[10] + Gx1[119]*dOld[11];
dNew[10] = + Gx1[120]*dOld[0] + Gx1[121]*dOld[1] + Gx1[122]*dOld[2] + Gx1[123]*dOld[3] + Gx1[124]*dOld[4] + Gx1[125]*dOld[5] + Gx1[126]*dOld[6] + Gx1[127]*dOld[7] + Gx1[128]*dOld[8] + Gx1[129]*dOld[9] + Gx1[130]*dOld[10] + Gx1[131]*dOld[11];
dNew[11] = + Gx1[132]*dOld[0] + Gx1[133]*dOld[1] + Gx1[134]*dOld[2] + Gx1[135]*dOld[3] + Gx1[136]*dOld[4] + Gx1[137]*dOld[5] + Gx1[138]*dOld[6] + Gx1[139]*dOld[7] + Gx1[140]*dOld[8] + Gx1[141]*dOld[9] + Gx1[142]*dOld[10] + Gx1[143]*dOld[11];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3] + acadoWorkspace.QN1[4]*dOld[4] + acadoWorkspace.QN1[5]*dOld[5] + acadoWorkspace.QN1[6]*dOld[6] + acadoWorkspace.QN1[7]*dOld[7] + acadoWorkspace.QN1[8]*dOld[8] + acadoWorkspace.QN1[9]*dOld[9] + acadoWorkspace.QN1[10]*dOld[10] + acadoWorkspace.QN1[11]*dOld[11];
dNew[1] = + acadoWorkspace.QN1[12]*dOld[0] + acadoWorkspace.QN1[13]*dOld[1] + acadoWorkspace.QN1[14]*dOld[2] + acadoWorkspace.QN1[15]*dOld[3] + acadoWorkspace.QN1[16]*dOld[4] + acadoWorkspace.QN1[17]*dOld[5] + acadoWorkspace.QN1[18]*dOld[6] + acadoWorkspace.QN1[19]*dOld[7] + acadoWorkspace.QN1[20]*dOld[8] + acadoWorkspace.QN1[21]*dOld[9] + acadoWorkspace.QN1[22]*dOld[10] + acadoWorkspace.QN1[23]*dOld[11];
dNew[2] = + acadoWorkspace.QN1[24]*dOld[0] + acadoWorkspace.QN1[25]*dOld[1] + acadoWorkspace.QN1[26]*dOld[2] + acadoWorkspace.QN1[27]*dOld[3] + acadoWorkspace.QN1[28]*dOld[4] + acadoWorkspace.QN1[29]*dOld[5] + acadoWorkspace.QN1[30]*dOld[6] + acadoWorkspace.QN1[31]*dOld[7] + acadoWorkspace.QN1[32]*dOld[8] + acadoWorkspace.QN1[33]*dOld[9] + acadoWorkspace.QN1[34]*dOld[10] + acadoWorkspace.QN1[35]*dOld[11];
dNew[3] = + acadoWorkspace.QN1[36]*dOld[0] + acadoWorkspace.QN1[37]*dOld[1] + acadoWorkspace.QN1[38]*dOld[2] + acadoWorkspace.QN1[39]*dOld[3] + acadoWorkspace.QN1[40]*dOld[4] + acadoWorkspace.QN1[41]*dOld[5] + acadoWorkspace.QN1[42]*dOld[6] + acadoWorkspace.QN1[43]*dOld[7] + acadoWorkspace.QN1[44]*dOld[8] + acadoWorkspace.QN1[45]*dOld[9] + acadoWorkspace.QN1[46]*dOld[10] + acadoWorkspace.QN1[47]*dOld[11];
dNew[4] = + acadoWorkspace.QN1[48]*dOld[0] + acadoWorkspace.QN1[49]*dOld[1] + acadoWorkspace.QN1[50]*dOld[2] + acadoWorkspace.QN1[51]*dOld[3] + acadoWorkspace.QN1[52]*dOld[4] + acadoWorkspace.QN1[53]*dOld[5] + acadoWorkspace.QN1[54]*dOld[6] + acadoWorkspace.QN1[55]*dOld[7] + acadoWorkspace.QN1[56]*dOld[8] + acadoWorkspace.QN1[57]*dOld[9] + acadoWorkspace.QN1[58]*dOld[10] + acadoWorkspace.QN1[59]*dOld[11];
dNew[5] = + acadoWorkspace.QN1[60]*dOld[0] + acadoWorkspace.QN1[61]*dOld[1] + acadoWorkspace.QN1[62]*dOld[2] + acadoWorkspace.QN1[63]*dOld[3] + acadoWorkspace.QN1[64]*dOld[4] + acadoWorkspace.QN1[65]*dOld[5] + acadoWorkspace.QN1[66]*dOld[6] + acadoWorkspace.QN1[67]*dOld[7] + acadoWorkspace.QN1[68]*dOld[8] + acadoWorkspace.QN1[69]*dOld[9] + acadoWorkspace.QN1[70]*dOld[10] + acadoWorkspace.QN1[71]*dOld[11];
dNew[6] = + acadoWorkspace.QN1[72]*dOld[0] + acadoWorkspace.QN1[73]*dOld[1] + acadoWorkspace.QN1[74]*dOld[2] + acadoWorkspace.QN1[75]*dOld[3] + acadoWorkspace.QN1[76]*dOld[4] + acadoWorkspace.QN1[77]*dOld[5] + acadoWorkspace.QN1[78]*dOld[6] + acadoWorkspace.QN1[79]*dOld[7] + acadoWorkspace.QN1[80]*dOld[8] + acadoWorkspace.QN1[81]*dOld[9] + acadoWorkspace.QN1[82]*dOld[10] + acadoWorkspace.QN1[83]*dOld[11];
dNew[7] = + acadoWorkspace.QN1[84]*dOld[0] + acadoWorkspace.QN1[85]*dOld[1] + acadoWorkspace.QN1[86]*dOld[2] + acadoWorkspace.QN1[87]*dOld[3] + acadoWorkspace.QN1[88]*dOld[4] + acadoWorkspace.QN1[89]*dOld[5] + acadoWorkspace.QN1[90]*dOld[6] + acadoWorkspace.QN1[91]*dOld[7] + acadoWorkspace.QN1[92]*dOld[8] + acadoWorkspace.QN1[93]*dOld[9] + acadoWorkspace.QN1[94]*dOld[10] + acadoWorkspace.QN1[95]*dOld[11];
dNew[8] = + acadoWorkspace.QN1[96]*dOld[0] + acadoWorkspace.QN1[97]*dOld[1] + acadoWorkspace.QN1[98]*dOld[2] + acadoWorkspace.QN1[99]*dOld[3] + acadoWorkspace.QN1[100]*dOld[4] + acadoWorkspace.QN1[101]*dOld[5] + acadoWorkspace.QN1[102]*dOld[6] + acadoWorkspace.QN1[103]*dOld[7] + acadoWorkspace.QN1[104]*dOld[8] + acadoWorkspace.QN1[105]*dOld[9] + acadoWorkspace.QN1[106]*dOld[10] + acadoWorkspace.QN1[107]*dOld[11];
dNew[9] = + acadoWorkspace.QN1[108]*dOld[0] + acadoWorkspace.QN1[109]*dOld[1] + acadoWorkspace.QN1[110]*dOld[2] + acadoWorkspace.QN1[111]*dOld[3] + acadoWorkspace.QN1[112]*dOld[4] + acadoWorkspace.QN1[113]*dOld[5] + acadoWorkspace.QN1[114]*dOld[6] + acadoWorkspace.QN1[115]*dOld[7] + acadoWorkspace.QN1[116]*dOld[8] + acadoWorkspace.QN1[117]*dOld[9] + acadoWorkspace.QN1[118]*dOld[10] + acadoWorkspace.QN1[119]*dOld[11];
dNew[10] = + acadoWorkspace.QN1[120]*dOld[0] + acadoWorkspace.QN1[121]*dOld[1] + acadoWorkspace.QN1[122]*dOld[2] + acadoWorkspace.QN1[123]*dOld[3] + acadoWorkspace.QN1[124]*dOld[4] + acadoWorkspace.QN1[125]*dOld[5] + acadoWorkspace.QN1[126]*dOld[6] + acadoWorkspace.QN1[127]*dOld[7] + acadoWorkspace.QN1[128]*dOld[8] + acadoWorkspace.QN1[129]*dOld[9] + acadoWorkspace.QN1[130]*dOld[10] + acadoWorkspace.QN1[131]*dOld[11];
dNew[11] = + acadoWorkspace.QN1[132]*dOld[0] + acadoWorkspace.QN1[133]*dOld[1] + acadoWorkspace.QN1[134]*dOld[2] + acadoWorkspace.QN1[135]*dOld[3] + acadoWorkspace.QN1[136]*dOld[4] + acadoWorkspace.QN1[137]*dOld[5] + acadoWorkspace.QN1[138]*dOld[6] + acadoWorkspace.QN1[139]*dOld[7] + acadoWorkspace.QN1[140]*dOld[8] + acadoWorkspace.QN1[141]*dOld[9] + acadoWorkspace.QN1[142]*dOld[10] + acadoWorkspace.QN1[143]*dOld[11];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9];
RDy1[1] = + R2[10]*Dy1[0] + R2[11]*Dy1[1] + R2[12]*Dy1[2] + R2[13]*Dy1[3] + R2[14]*Dy1[4] + R2[15]*Dy1[5] + R2[16]*Dy1[6] + R2[17]*Dy1[7] + R2[18]*Dy1[8] + R2[19]*Dy1[9];
RDy1[2] = + R2[20]*Dy1[0] + R2[21]*Dy1[1] + R2[22]*Dy1[2] + R2[23]*Dy1[3] + R2[24]*Dy1[4] + R2[25]*Dy1[5] + R2[26]*Dy1[6] + R2[27]*Dy1[7] + R2[28]*Dy1[8] + R2[29]*Dy1[9];
RDy1[3] = + R2[30]*Dy1[0] + R2[31]*Dy1[1] + R2[32]*Dy1[2] + R2[33]*Dy1[3] + R2[34]*Dy1[4] + R2[35]*Dy1[5] + R2[36]*Dy1[6] + R2[37]*Dy1[7] + R2[38]*Dy1[8] + R2[39]*Dy1[9];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9];
QDy1[1] = + Q2[10]*Dy1[0] + Q2[11]*Dy1[1] + Q2[12]*Dy1[2] + Q2[13]*Dy1[3] + Q2[14]*Dy1[4] + Q2[15]*Dy1[5] + Q2[16]*Dy1[6] + Q2[17]*Dy1[7] + Q2[18]*Dy1[8] + Q2[19]*Dy1[9];
QDy1[2] = + Q2[20]*Dy1[0] + Q2[21]*Dy1[1] + Q2[22]*Dy1[2] + Q2[23]*Dy1[3] + Q2[24]*Dy1[4] + Q2[25]*Dy1[5] + Q2[26]*Dy1[6] + Q2[27]*Dy1[7] + Q2[28]*Dy1[8] + Q2[29]*Dy1[9];
QDy1[3] = + Q2[30]*Dy1[0] + Q2[31]*Dy1[1] + Q2[32]*Dy1[2] + Q2[33]*Dy1[3] + Q2[34]*Dy1[4] + Q2[35]*Dy1[5] + Q2[36]*Dy1[6] + Q2[37]*Dy1[7] + Q2[38]*Dy1[8] + Q2[39]*Dy1[9];
QDy1[4] = + Q2[40]*Dy1[0] + Q2[41]*Dy1[1] + Q2[42]*Dy1[2] + Q2[43]*Dy1[3] + Q2[44]*Dy1[4] + Q2[45]*Dy1[5] + Q2[46]*Dy1[6] + Q2[47]*Dy1[7] + Q2[48]*Dy1[8] + Q2[49]*Dy1[9];
QDy1[5] = + Q2[50]*Dy1[0] + Q2[51]*Dy1[1] + Q2[52]*Dy1[2] + Q2[53]*Dy1[3] + Q2[54]*Dy1[4] + Q2[55]*Dy1[5] + Q2[56]*Dy1[6] + Q2[57]*Dy1[7] + Q2[58]*Dy1[8] + Q2[59]*Dy1[9];
QDy1[6] = + Q2[60]*Dy1[0] + Q2[61]*Dy1[1] + Q2[62]*Dy1[2] + Q2[63]*Dy1[3] + Q2[64]*Dy1[4] + Q2[65]*Dy1[5] + Q2[66]*Dy1[6] + Q2[67]*Dy1[7] + Q2[68]*Dy1[8] + Q2[69]*Dy1[9];
QDy1[7] = + Q2[70]*Dy1[0] + Q2[71]*Dy1[1] + Q2[72]*Dy1[2] + Q2[73]*Dy1[3] + Q2[74]*Dy1[4] + Q2[75]*Dy1[5] + Q2[76]*Dy1[6] + Q2[77]*Dy1[7] + Q2[78]*Dy1[8] + Q2[79]*Dy1[9];
QDy1[8] = + Q2[80]*Dy1[0] + Q2[81]*Dy1[1] + Q2[82]*Dy1[2] + Q2[83]*Dy1[3] + Q2[84]*Dy1[4] + Q2[85]*Dy1[5] + Q2[86]*Dy1[6] + Q2[87]*Dy1[7] + Q2[88]*Dy1[8] + Q2[89]*Dy1[9];
QDy1[9] = + Q2[90]*Dy1[0] + Q2[91]*Dy1[1] + Q2[92]*Dy1[2] + Q2[93]*Dy1[3] + Q2[94]*Dy1[4] + Q2[95]*Dy1[5] + Q2[96]*Dy1[6] + Q2[97]*Dy1[7] + Q2[98]*Dy1[8] + Q2[99]*Dy1[9];
QDy1[10] = + Q2[100]*Dy1[0] + Q2[101]*Dy1[1] + Q2[102]*Dy1[2] + Q2[103]*Dy1[3] + Q2[104]*Dy1[4] + Q2[105]*Dy1[5] + Q2[106]*Dy1[6] + Q2[107]*Dy1[7] + Q2[108]*Dy1[8] + Q2[109]*Dy1[9];
QDy1[11] = + Q2[110]*Dy1[0] + Q2[111]*Dy1[1] + Q2[112]*Dy1[2] + Q2[113]*Dy1[3] + Q2[114]*Dy1[4] + Q2[115]*Dy1[5] + Q2[116]*Dy1[6] + Q2[117]*Dy1[7] + Q2[118]*Dy1[8] + Q2[119]*Dy1[9];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[4]*QDy1[1] + E1[8]*QDy1[2] + E1[12]*QDy1[3] + E1[16]*QDy1[4] + E1[20]*QDy1[5] + E1[24]*QDy1[6] + E1[28]*QDy1[7] + E1[32]*QDy1[8] + E1[36]*QDy1[9] + E1[40]*QDy1[10] + E1[44]*QDy1[11];
U1[1] += + E1[1]*QDy1[0] + E1[5]*QDy1[1] + E1[9]*QDy1[2] + E1[13]*QDy1[3] + E1[17]*QDy1[4] + E1[21]*QDy1[5] + E1[25]*QDy1[6] + E1[29]*QDy1[7] + E1[33]*QDy1[8] + E1[37]*QDy1[9] + E1[41]*QDy1[10] + E1[45]*QDy1[11];
U1[2] += + E1[2]*QDy1[0] + E1[6]*QDy1[1] + E1[10]*QDy1[2] + E1[14]*QDy1[3] + E1[18]*QDy1[4] + E1[22]*QDy1[5] + E1[26]*QDy1[6] + E1[30]*QDy1[7] + E1[34]*QDy1[8] + E1[38]*QDy1[9] + E1[42]*QDy1[10] + E1[46]*QDy1[11];
U1[3] += + E1[3]*QDy1[0] + E1[7]*QDy1[1] + E1[11]*QDy1[2] + E1[15]*QDy1[3] + E1[19]*QDy1[4] + E1[23]*QDy1[5] + E1[27]*QDy1[6] + E1[31]*QDy1[7] + E1[35]*QDy1[8] + E1[39]*QDy1[9] + E1[43]*QDy1[10] + E1[47]*QDy1[11];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[4]*Gx1[12] + E1[8]*Gx1[24] + E1[12]*Gx1[36] + E1[16]*Gx1[48] + E1[20]*Gx1[60] + E1[24]*Gx1[72] + E1[28]*Gx1[84] + E1[32]*Gx1[96] + E1[36]*Gx1[108] + E1[40]*Gx1[120] + E1[44]*Gx1[132];
H101[1] += + E1[0]*Gx1[1] + E1[4]*Gx1[13] + E1[8]*Gx1[25] + E1[12]*Gx1[37] + E1[16]*Gx1[49] + E1[20]*Gx1[61] + E1[24]*Gx1[73] + E1[28]*Gx1[85] + E1[32]*Gx1[97] + E1[36]*Gx1[109] + E1[40]*Gx1[121] + E1[44]*Gx1[133];
H101[2] += + E1[0]*Gx1[2] + E1[4]*Gx1[14] + E1[8]*Gx1[26] + E1[12]*Gx1[38] + E1[16]*Gx1[50] + E1[20]*Gx1[62] + E1[24]*Gx1[74] + E1[28]*Gx1[86] + E1[32]*Gx1[98] + E1[36]*Gx1[110] + E1[40]*Gx1[122] + E1[44]*Gx1[134];
H101[3] += + E1[0]*Gx1[3] + E1[4]*Gx1[15] + E1[8]*Gx1[27] + E1[12]*Gx1[39] + E1[16]*Gx1[51] + E1[20]*Gx1[63] + E1[24]*Gx1[75] + E1[28]*Gx1[87] + E1[32]*Gx1[99] + E1[36]*Gx1[111] + E1[40]*Gx1[123] + E1[44]*Gx1[135];
H101[4] += + E1[0]*Gx1[4] + E1[4]*Gx1[16] + E1[8]*Gx1[28] + E1[12]*Gx1[40] + E1[16]*Gx1[52] + E1[20]*Gx1[64] + E1[24]*Gx1[76] + E1[28]*Gx1[88] + E1[32]*Gx1[100] + E1[36]*Gx1[112] + E1[40]*Gx1[124] + E1[44]*Gx1[136];
H101[5] += + E1[0]*Gx1[5] + E1[4]*Gx1[17] + E1[8]*Gx1[29] + E1[12]*Gx1[41] + E1[16]*Gx1[53] + E1[20]*Gx1[65] + E1[24]*Gx1[77] + E1[28]*Gx1[89] + E1[32]*Gx1[101] + E1[36]*Gx1[113] + E1[40]*Gx1[125] + E1[44]*Gx1[137];
H101[6] += + E1[0]*Gx1[6] + E1[4]*Gx1[18] + E1[8]*Gx1[30] + E1[12]*Gx1[42] + E1[16]*Gx1[54] + E1[20]*Gx1[66] + E1[24]*Gx1[78] + E1[28]*Gx1[90] + E1[32]*Gx1[102] + E1[36]*Gx1[114] + E1[40]*Gx1[126] + E1[44]*Gx1[138];
H101[7] += + E1[0]*Gx1[7] + E1[4]*Gx1[19] + E1[8]*Gx1[31] + E1[12]*Gx1[43] + E1[16]*Gx1[55] + E1[20]*Gx1[67] + E1[24]*Gx1[79] + E1[28]*Gx1[91] + E1[32]*Gx1[103] + E1[36]*Gx1[115] + E1[40]*Gx1[127] + E1[44]*Gx1[139];
H101[8] += + E1[0]*Gx1[8] + E1[4]*Gx1[20] + E1[8]*Gx1[32] + E1[12]*Gx1[44] + E1[16]*Gx1[56] + E1[20]*Gx1[68] + E1[24]*Gx1[80] + E1[28]*Gx1[92] + E1[32]*Gx1[104] + E1[36]*Gx1[116] + E1[40]*Gx1[128] + E1[44]*Gx1[140];
H101[9] += + E1[0]*Gx1[9] + E1[4]*Gx1[21] + E1[8]*Gx1[33] + E1[12]*Gx1[45] + E1[16]*Gx1[57] + E1[20]*Gx1[69] + E1[24]*Gx1[81] + E1[28]*Gx1[93] + E1[32]*Gx1[105] + E1[36]*Gx1[117] + E1[40]*Gx1[129] + E1[44]*Gx1[141];
H101[10] += + E1[0]*Gx1[10] + E1[4]*Gx1[22] + E1[8]*Gx1[34] + E1[12]*Gx1[46] + E1[16]*Gx1[58] + E1[20]*Gx1[70] + E1[24]*Gx1[82] + E1[28]*Gx1[94] + E1[32]*Gx1[106] + E1[36]*Gx1[118] + E1[40]*Gx1[130] + E1[44]*Gx1[142];
H101[11] += + E1[0]*Gx1[11] + E1[4]*Gx1[23] + E1[8]*Gx1[35] + E1[12]*Gx1[47] + E1[16]*Gx1[59] + E1[20]*Gx1[71] + E1[24]*Gx1[83] + E1[28]*Gx1[95] + E1[32]*Gx1[107] + E1[36]*Gx1[119] + E1[40]*Gx1[131] + E1[44]*Gx1[143];
H101[12] += + E1[1]*Gx1[0] + E1[5]*Gx1[12] + E1[9]*Gx1[24] + E1[13]*Gx1[36] + E1[17]*Gx1[48] + E1[21]*Gx1[60] + E1[25]*Gx1[72] + E1[29]*Gx1[84] + E1[33]*Gx1[96] + E1[37]*Gx1[108] + E1[41]*Gx1[120] + E1[45]*Gx1[132];
H101[13] += + E1[1]*Gx1[1] + E1[5]*Gx1[13] + E1[9]*Gx1[25] + E1[13]*Gx1[37] + E1[17]*Gx1[49] + E1[21]*Gx1[61] + E1[25]*Gx1[73] + E1[29]*Gx1[85] + E1[33]*Gx1[97] + E1[37]*Gx1[109] + E1[41]*Gx1[121] + E1[45]*Gx1[133];
H101[14] += + E1[1]*Gx1[2] + E1[5]*Gx1[14] + E1[9]*Gx1[26] + E1[13]*Gx1[38] + E1[17]*Gx1[50] + E1[21]*Gx1[62] + E1[25]*Gx1[74] + E1[29]*Gx1[86] + E1[33]*Gx1[98] + E1[37]*Gx1[110] + E1[41]*Gx1[122] + E1[45]*Gx1[134];
H101[15] += + E1[1]*Gx1[3] + E1[5]*Gx1[15] + E1[9]*Gx1[27] + E1[13]*Gx1[39] + E1[17]*Gx1[51] + E1[21]*Gx1[63] + E1[25]*Gx1[75] + E1[29]*Gx1[87] + E1[33]*Gx1[99] + E1[37]*Gx1[111] + E1[41]*Gx1[123] + E1[45]*Gx1[135];
H101[16] += + E1[1]*Gx1[4] + E1[5]*Gx1[16] + E1[9]*Gx1[28] + E1[13]*Gx1[40] + E1[17]*Gx1[52] + E1[21]*Gx1[64] + E1[25]*Gx1[76] + E1[29]*Gx1[88] + E1[33]*Gx1[100] + E1[37]*Gx1[112] + E1[41]*Gx1[124] + E1[45]*Gx1[136];
H101[17] += + E1[1]*Gx1[5] + E1[5]*Gx1[17] + E1[9]*Gx1[29] + E1[13]*Gx1[41] + E1[17]*Gx1[53] + E1[21]*Gx1[65] + E1[25]*Gx1[77] + E1[29]*Gx1[89] + E1[33]*Gx1[101] + E1[37]*Gx1[113] + E1[41]*Gx1[125] + E1[45]*Gx1[137];
H101[18] += + E1[1]*Gx1[6] + E1[5]*Gx1[18] + E1[9]*Gx1[30] + E1[13]*Gx1[42] + E1[17]*Gx1[54] + E1[21]*Gx1[66] + E1[25]*Gx1[78] + E1[29]*Gx1[90] + E1[33]*Gx1[102] + E1[37]*Gx1[114] + E1[41]*Gx1[126] + E1[45]*Gx1[138];
H101[19] += + E1[1]*Gx1[7] + E1[5]*Gx1[19] + E1[9]*Gx1[31] + E1[13]*Gx1[43] + E1[17]*Gx1[55] + E1[21]*Gx1[67] + E1[25]*Gx1[79] + E1[29]*Gx1[91] + E1[33]*Gx1[103] + E1[37]*Gx1[115] + E1[41]*Gx1[127] + E1[45]*Gx1[139];
H101[20] += + E1[1]*Gx1[8] + E1[5]*Gx1[20] + E1[9]*Gx1[32] + E1[13]*Gx1[44] + E1[17]*Gx1[56] + E1[21]*Gx1[68] + E1[25]*Gx1[80] + E1[29]*Gx1[92] + E1[33]*Gx1[104] + E1[37]*Gx1[116] + E1[41]*Gx1[128] + E1[45]*Gx1[140];
H101[21] += + E1[1]*Gx1[9] + E1[5]*Gx1[21] + E1[9]*Gx1[33] + E1[13]*Gx1[45] + E1[17]*Gx1[57] + E1[21]*Gx1[69] + E1[25]*Gx1[81] + E1[29]*Gx1[93] + E1[33]*Gx1[105] + E1[37]*Gx1[117] + E1[41]*Gx1[129] + E1[45]*Gx1[141];
H101[22] += + E1[1]*Gx1[10] + E1[5]*Gx1[22] + E1[9]*Gx1[34] + E1[13]*Gx1[46] + E1[17]*Gx1[58] + E1[21]*Gx1[70] + E1[25]*Gx1[82] + E1[29]*Gx1[94] + E1[33]*Gx1[106] + E1[37]*Gx1[118] + E1[41]*Gx1[130] + E1[45]*Gx1[142];
H101[23] += + E1[1]*Gx1[11] + E1[5]*Gx1[23] + E1[9]*Gx1[35] + E1[13]*Gx1[47] + E1[17]*Gx1[59] + E1[21]*Gx1[71] + E1[25]*Gx1[83] + E1[29]*Gx1[95] + E1[33]*Gx1[107] + E1[37]*Gx1[119] + E1[41]*Gx1[131] + E1[45]*Gx1[143];
H101[24] += + E1[2]*Gx1[0] + E1[6]*Gx1[12] + E1[10]*Gx1[24] + E1[14]*Gx1[36] + E1[18]*Gx1[48] + E1[22]*Gx1[60] + E1[26]*Gx1[72] + E1[30]*Gx1[84] + E1[34]*Gx1[96] + E1[38]*Gx1[108] + E1[42]*Gx1[120] + E1[46]*Gx1[132];
H101[25] += + E1[2]*Gx1[1] + E1[6]*Gx1[13] + E1[10]*Gx1[25] + E1[14]*Gx1[37] + E1[18]*Gx1[49] + E1[22]*Gx1[61] + E1[26]*Gx1[73] + E1[30]*Gx1[85] + E1[34]*Gx1[97] + E1[38]*Gx1[109] + E1[42]*Gx1[121] + E1[46]*Gx1[133];
H101[26] += + E1[2]*Gx1[2] + E1[6]*Gx1[14] + E1[10]*Gx1[26] + E1[14]*Gx1[38] + E1[18]*Gx1[50] + E1[22]*Gx1[62] + E1[26]*Gx1[74] + E1[30]*Gx1[86] + E1[34]*Gx1[98] + E1[38]*Gx1[110] + E1[42]*Gx1[122] + E1[46]*Gx1[134];
H101[27] += + E1[2]*Gx1[3] + E1[6]*Gx1[15] + E1[10]*Gx1[27] + E1[14]*Gx1[39] + E1[18]*Gx1[51] + E1[22]*Gx1[63] + E1[26]*Gx1[75] + E1[30]*Gx1[87] + E1[34]*Gx1[99] + E1[38]*Gx1[111] + E1[42]*Gx1[123] + E1[46]*Gx1[135];
H101[28] += + E1[2]*Gx1[4] + E1[6]*Gx1[16] + E1[10]*Gx1[28] + E1[14]*Gx1[40] + E1[18]*Gx1[52] + E1[22]*Gx1[64] + E1[26]*Gx1[76] + E1[30]*Gx1[88] + E1[34]*Gx1[100] + E1[38]*Gx1[112] + E1[42]*Gx1[124] + E1[46]*Gx1[136];
H101[29] += + E1[2]*Gx1[5] + E1[6]*Gx1[17] + E1[10]*Gx1[29] + E1[14]*Gx1[41] + E1[18]*Gx1[53] + E1[22]*Gx1[65] + E1[26]*Gx1[77] + E1[30]*Gx1[89] + E1[34]*Gx1[101] + E1[38]*Gx1[113] + E1[42]*Gx1[125] + E1[46]*Gx1[137];
H101[30] += + E1[2]*Gx1[6] + E1[6]*Gx1[18] + E1[10]*Gx1[30] + E1[14]*Gx1[42] + E1[18]*Gx1[54] + E1[22]*Gx1[66] + E1[26]*Gx1[78] + E1[30]*Gx1[90] + E1[34]*Gx1[102] + E1[38]*Gx1[114] + E1[42]*Gx1[126] + E1[46]*Gx1[138];
H101[31] += + E1[2]*Gx1[7] + E1[6]*Gx1[19] + E1[10]*Gx1[31] + E1[14]*Gx1[43] + E1[18]*Gx1[55] + E1[22]*Gx1[67] + E1[26]*Gx1[79] + E1[30]*Gx1[91] + E1[34]*Gx1[103] + E1[38]*Gx1[115] + E1[42]*Gx1[127] + E1[46]*Gx1[139];
H101[32] += + E1[2]*Gx1[8] + E1[6]*Gx1[20] + E1[10]*Gx1[32] + E1[14]*Gx1[44] + E1[18]*Gx1[56] + E1[22]*Gx1[68] + E1[26]*Gx1[80] + E1[30]*Gx1[92] + E1[34]*Gx1[104] + E1[38]*Gx1[116] + E1[42]*Gx1[128] + E1[46]*Gx1[140];
H101[33] += + E1[2]*Gx1[9] + E1[6]*Gx1[21] + E1[10]*Gx1[33] + E1[14]*Gx1[45] + E1[18]*Gx1[57] + E1[22]*Gx1[69] + E1[26]*Gx1[81] + E1[30]*Gx1[93] + E1[34]*Gx1[105] + E1[38]*Gx1[117] + E1[42]*Gx1[129] + E1[46]*Gx1[141];
H101[34] += + E1[2]*Gx1[10] + E1[6]*Gx1[22] + E1[10]*Gx1[34] + E1[14]*Gx1[46] + E1[18]*Gx1[58] + E1[22]*Gx1[70] + E1[26]*Gx1[82] + E1[30]*Gx1[94] + E1[34]*Gx1[106] + E1[38]*Gx1[118] + E1[42]*Gx1[130] + E1[46]*Gx1[142];
H101[35] += + E1[2]*Gx1[11] + E1[6]*Gx1[23] + E1[10]*Gx1[35] + E1[14]*Gx1[47] + E1[18]*Gx1[59] + E1[22]*Gx1[71] + E1[26]*Gx1[83] + E1[30]*Gx1[95] + E1[34]*Gx1[107] + E1[38]*Gx1[119] + E1[42]*Gx1[131] + E1[46]*Gx1[143];
H101[36] += + E1[3]*Gx1[0] + E1[7]*Gx1[12] + E1[11]*Gx1[24] + E1[15]*Gx1[36] + E1[19]*Gx1[48] + E1[23]*Gx1[60] + E1[27]*Gx1[72] + E1[31]*Gx1[84] + E1[35]*Gx1[96] + E1[39]*Gx1[108] + E1[43]*Gx1[120] + E1[47]*Gx1[132];
H101[37] += + E1[3]*Gx1[1] + E1[7]*Gx1[13] + E1[11]*Gx1[25] + E1[15]*Gx1[37] + E1[19]*Gx1[49] + E1[23]*Gx1[61] + E1[27]*Gx1[73] + E1[31]*Gx1[85] + E1[35]*Gx1[97] + E1[39]*Gx1[109] + E1[43]*Gx1[121] + E1[47]*Gx1[133];
H101[38] += + E1[3]*Gx1[2] + E1[7]*Gx1[14] + E1[11]*Gx1[26] + E1[15]*Gx1[38] + E1[19]*Gx1[50] + E1[23]*Gx1[62] + E1[27]*Gx1[74] + E1[31]*Gx1[86] + E1[35]*Gx1[98] + E1[39]*Gx1[110] + E1[43]*Gx1[122] + E1[47]*Gx1[134];
H101[39] += + E1[3]*Gx1[3] + E1[7]*Gx1[15] + E1[11]*Gx1[27] + E1[15]*Gx1[39] + E1[19]*Gx1[51] + E1[23]*Gx1[63] + E1[27]*Gx1[75] + E1[31]*Gx1[87] + E1[35]*Gx1[99] + E1[39]*Gx1[111] + E1[43]*Gx1[123] + E1[47]*Gx1[135];
H101[40] += + E1[3]*Gx1[4] + E1[7]*Gx1[16] + E1[11]*Gx1[28] + E1[15]*Gx1[40] + E1[19]*Gx1[52] + E1[23]*Gx1[64] + E1[27]*Gx1[76] + E1[31]*Gx1[88] + E1[35]*Gx1[100] + E1[39]*Gx1[112] + E1[43]*Gx1[124] + E1[47]*Gx1[136];
H101[41] += + E1[3]*Gx1[5] + E1[7]*Gx1[17] + E1[11]*Gx1[29] + E1[15]*Gx1[41] + E1[19]*Gx1[53] + E1[23]*Gx1[65] + E1[27]*Gx1[77] + E1[31]*Gx1[89] + E1[35]*Gx1[101] + E1[39]*Gx1[113] + E1[43]*Gx1[125] + E1[47]*Gx1[137];
H101[42] += + E1[3]*Gx1[6] + E1[7]*Gx1[18] + E1[11]*Gx1[30] + E1[15]*Gx1[42] + E1[19]*Gx1[54] + E1[23]*Gx1[66] + E1[27]*Gx1[78] + E1[31]*Gx1[90] + E1[35]*Gx1[102] + E1[39]*Gx1[114] + E1[43]*Gx1[126] + E1[47]*Gx1[138];
H101[43] += + E1[3]*Gx1[7] + E1[7]*Gx1[19] + E1[11]*Gx1[31] + E1[15]*Gx1[43] + E1[19]*Gx1[55] + E1[23]*Gx1[67] + E1[27]*Gx1[79] + E1[31]*Gx1[91] + E1[35]*Gx1[103] + E1[39]*Gx1[115] + E1[43]*Gx1[127] + E1[47]*Gx1[139];
H101[44] += + E1[3]*Gx1[8] + E1[7]*Gx1[20] + E1[11]*Gx1[32] + E1[15]*Gx1[44] + E1[19]*Gx1[56] + E1[23]*Gx1[68] + E1[27]*Gx1[80] + E1[31]*Gx1[92] + E1[35]*Gx1[104] + E1[39]*Gx1[116] + E1[43]*Gx1[128] + E1[47]*Gx1[140];
H101[45] += + E1[3]*Gx1[9] + E1[7]*Gx1[21] + E1[11]*Gx1[33] + E1[15]*Gx1[45] + E1[19]*Gx1[57] + E1[23]*Gx1[69] + E1[27]*Gx1[81] + E1[31]*Gx1[93] + E1[35]*Gx1[105] + E1[39]*Gx1[117] + E1[43]*Gx1[129] + E1[47]*Gx1[141];
H101[46] += + E1[3]*Gx1[10] + E1[7]*Gx1[22] + E1[11]*Gx1[34] + E1[15]*Gx1[46] + E1[19]*Gx1[58] + E1[23]*Gx1[70] + E1[27]*Gx1[82] + E1[31]*Gx1[94] + E1[35]*Gx1[106] + E1[39]*Gx1[118] + E1[43]*Gx1[130] + E1[47]*Gx1[142];
H101[47] += + E1[3]*Gx1[11] + E1[7]*Gx1[23] + E1[11]*Gx1[35] + E1[15]*Gx1[47] + E1[19]*Gx1[59] + E1[23]*Gx1[71] + E1[27]*Gx1[83] + E1[31]*Gx1[95] + E1[35]*Gx1[107] + E1[39]*Gx1[119] + E1[43]*Gx1[131] + E1[47]*Gx1[143];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 48; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2] + E1[3]*U1[3];
dNew[1] += + E1[4]*U1[0] + E1[5]*U1[1] + E1[6]*U1[2] + E1[7]*U1[3];
dNew[2] += + E1[8]*U1[0] + E1[9]*U1[1] + E1[10]*U1[2] + E1[11]*U1[3];
dNew[3] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2] + E1[15]*U1[3];
dNew[4] += + E1[16]*U1[0] + E1[17]*U1[1] + E1[18]*U1[2] + E1[19]*U1[3];
dNew[5] += + E1[20]*U1[0] + E1[21]*U1[1] + E1[22]*U1[2] + E1[23]*U1[3];
dNew[6] += + E1[24]*U1[0] + E1[25]*U1[1] + E1[26]*U1[2] + E1[27]*U1[3];
dNew[7] += + E1[28]*U1[0] + E1[29]*U1[1] + E1[30]*U1[2] + E1[31]*U1[3];
dNew[8] += + E1[32]*U1[0] + E1[33]*U1[1] + E1[34]*U1[2] + E1[35]*U1[3];
dNew[9] += + E1[36]*U1[0] + E1[37]*U1[1] + E1[38]*U1[2] + E1[39]*U1[3];
dNew[10] += + E1[40]*U1[0] + E1[41]*U1[1] + E1[42]*U1[2] + E1[43]*U1[3];
dNew[11] += + E1[44]*U1[0] + E1[45]*U1[1] + E1[46]*U1[2] + E1[47]*U1[3];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[12] + Hx[2]*Gx[24] + Hx[3]*Gx[36] + Hx[4]*Gx[48] + Hx[5]*Gx[60] + Hx[6]*Gx[72] + Hx[7]*Gx[84] + Hx[8]*Gx[96] + Hx[9]*Gx[108] + Hx[10]*Gx[120] + Hx[11]*Gx[132];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[13] + Hx[2]*Gx[25] + Hx[3]*Gx[37] + Hx[4]*Gx[49] + Hx[5]*Gx[61] + Hx[6]*Gx[73] + Hx[7]*Gx[85] + Hx[8]*Gx[97] + Hx[9]*Gx[109] + Hx[10]*Gx[121] + Hx[11]*Gx[133];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[14] + Hx[2]*Gx[26] + Hx[3]*Gx[38] + Hx[4]*Gx[50] + Hx[5]*Gx[62] + Hx[6]*Gx[74] + Hx[7]*Gx[86] + Hx[8]*Gx[98] + Hx[9]*Gx[110] + Hx[10]*Gx[122] + Hx[11]*Gx[134];
A01[3] = + Hx[0]*Gx[3] + Hx[1]*Gx[15] + Hx[2]*Gx[27] + Hx[3]*Gx[39] + Hx[4]*Gx[51] + Hx[5]*Gx[63] + Hx[6]*Gx[75] + Hx[7]*Gx[87] + Hx[8]*Gx[99] + Hx[9]*Gx[111] + Hx[10]*Gx[123] + Hx[11]*Gx[135];
A01[4] = + Hx[0]*Gx[4] + Hx[1]*Gx[16] + Hx[2]*Gx[28] + Hx[3]*Gx[40] + Hx[4]*Gx[52] + Hx[5]*Gx[64] + Hx[6]*Gx[76] + Hx[7]*Gx[88] + Hx[8]*Gx[100] + Hx[9]*Gx[112] + Hx[10]*Gx[124] + Hx[11]*Gx[136];
A01[5] = + Hx[0]*Gx[5] + Hx[1]*Gx[17] + Hx[2]*Gx[29] + Hx[3]*Gx[41] + Hx[4]*Gx[53] + Hx[5]*Gx[65] + Hx[6]*Gx[77] + Hx[7]*Gx[89] + Hx[8]*Gx[101] + Hx[9]*Gx[113] + Hx[10]*Gx[125] + Hx[11]*Gx[137];
A01[6] = + Hx[0]*Gx[6] + Hx[1]*Gx[18] + Hx[2]*Gx[30] + Hx[3]*Gx[42] + Hx[4]*Gx[54] + Hx[5]*Gx[66] + Hx[6]*Gx[78] + Hx[7]*Gx[90] + Hx[8]*Gx[102] + Hx[9]*Gx[114] + Hx[10]*Gx[126] + Hx[11]*Gx[138];
A01[7] = + Hx[0]*Gx[7] + Hx[1]*Gx[19] + Hx[2]*Gx[31] + Hx[3]*Gx[43] + Hx[4]*Gx[55] + Hx[5]*Gx[67] + Hx[6]*Gx[79] + Hx[7]*Gx[91] + Hx[8]*Gx[103] + Hx[9]*Gx[115] + Hx[10]*Gx[127] + Hx[11]*Gx[139];
A01[8] = + Hx[0]*Gx[8] + Hx[1]*Gx[20] + Hx[2]*Gx[32] + Hx[3]*Gx[44] + Hx[4]*Gx[56] + Hx[5]*Gx[68] + Hx[6]*Gx[80] + Hx[7]*Gx[92] + Hx[8]*Gx[104] + Hx[9]*Gx[116] + Hx[10]*Gx[128] + Hx[11]*Gx[140];
A01[9] = + Hx[0]*Gx[9] + Hx[1]*Gx[21] + Hx[2]*Gx[33] + Hx[3]*Gx[45] + Hx[4]*Gx[57] + Hx[5]*Gx[69] + Hx[6]*Gx[81] + Hx[7]*Gx[93] + Hx[8]*Gx[105] + Hx[9]*Gx[117] + Hx[10]*Gx[129] + Hx[11]*Gx[141];
A01[10] = + Hx[0]*Gx[10] + Hx[1]*Gx[22] + Hx[2]*Gx[34] + Hx[3]*Gx[46] + Hx[4]*Gx[58] + Hx[5]*Gx[70] + Hx[6]*Gx[82] + Hx[7]*Gx[94] + Hx[8]*Gx[106] + Hx[9]*Gx[118] + Hx[10]*Gx[130] + Hx[11]*Gx[142];
A01[11] = + Hx[0]*Gx[11] + Hx[1]*Gx[23] + Hx[2]*Gx[35] + Hx[3]*Gx[47] + Hx[4]*Gx[59] + Hx[5]*Gx[71] + Hx[6]*Gx[83] + Hx[7]*Gx[95] + Hx[8]*Gx[107] + Hx[9]*Gx[119] + Hx[10]*Gx[131] + Hx[11]*Gx[143];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 120 + 10800) + (col * 4)] = + Hx[0]*E[0] + Hx[1]*E[4] + Hx[2]*E[8] + Hx[3]*E[12] + Hx[4]*E[16] + Hx[5]*E[20] + Hx[6]*E[24] + Hx[7]*E[28] + Hx[8]*E[32] + Hx[9]*E[36] + Hx[10]*E[40] + Hx[11]*E[44];
acadoWorkspace.A[(row * 120 + 10800) + (col * 4 + 1)] = + Hx[0]*E[1] + Hx[1]*E[5] + Hx[2]*E[9] + Hx[3]*E[13] + Hx[4]*E[17] + Hx[5]*E[21] + Hx[6]*E[25] + Hx[7]*E[29] + Hx[8]*E[33] + Hx[9]*E[37] + Hx[10]*E[41] + Hx[11]*E[45];
acadoWorkspace.A[(row * 120 + 10800) + (col * 4 + 2)] = + Hx[0]*E[2] + Hx[1]*E[6] + Hx[2]*E[10] + Hx[3]*E[14] + Hx[4]*E[18] + Hx[5]*E[22] + Hx[6]*E[26] + Hx[7]*E[30] + Hx[8]*E[34] + Hx[9]*E[38] + Hx[10]*E[42] + Hx[11]*E[46];
acadoWorkspace.A[(row * 120 + 10800) + (col * 4 + 3)] = + Hx[0]*E[3] + Hx[1]*E[7] + Hx[2]*E[11] + Hx[3]*E[15] + Hx[4]*E[19] + Hx[5]*E[23] + Hx[6]*E[27] + Hx[7]*E[31] + Hx[8]*E[35] + Hx[9]*E[39] + Hx[10]*E[43] + Hx[11]*E[47];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4] + Hx[5]*tmpd[5] + Hx[6]*tmpd[6] + Hx[7]*tmpd[7] + Hx[8]*tmpd[8] + Hx[9]*tmpd[9] + Hx[10]*tmpd[10] + Hx[11]*tmpd[11];
lbA[0] -= acadoWorkspace.evHxd[0];
ubA[0] -= acadoWorkspace.evHxd[0];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 28. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (sqrt(((((xd[0]-(real_t)(1.0000000000000000e+00))*(xd[0]-(real_t)(1.0000000000000000e+00)))+((xd[1]-(real_t)(1.0000000000000000e+00))*(xd[1]-(real_t)(1.0000000000000000e+00))))+((xd[2]-(real_t)(1.0000000000000000e+00))*(xd[2]-(real_t)(1.0000000000000000e+00))))));
a[1] = (xd[0]-(real_t)(1.0000000000000000e+00));
a[2] = (xd[0]-(real_t)(1.0000000000000000e+00));
a[3] = (a[1]+a[2]);
a[4] = (1.0/sqrt(((((xd[0]-(real_t)(1.0000000000000000e+00))*(xd[0]-(real_t)(1.0000000000000000e+00)))+((xd[1]-(real_t)(1.0000000000000000e+00))*(xd[1]-(real_t)(1.0000000000000000e+00))))+((xd[2]-(real_t)(1.0000000000000000e+00))*(xd[2]-(real_t)(1.0000000000000000e+00))))));
a[5] = (a[4]*(real_t)(5.0000000000000000e-01));
a[6] = (a[3]*a[5]);
a[7] = (xd[1]-(real_t)(1.0000000000000000e+00));
a[8] = (xd[1]-(real_t)(1.0000000000000000e+00));
a[9] = (a[7]+a[8]);
a[10] = (a[9]*a[5]);
a[11] = (xd[2]-(real_t)(1.0000000000000000e+00));
a[12] = (xd[2]-(real_t)(1.0000000000000000e+00));
a[13] = (a[11]+a[12]);
a[14] = (a[13]*a[5]);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (real_t)(0.0000000000000000e+00);
a[27] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = a[0];
out[1] = a[6];
out[2] = a[10];
out[3] = a[14];
out[4] = a[15];
out[5] = a[16];
out[6] = a[17];
out[7] = a[18];
out[8] = a[19];
out[9] = a[20];
out[10] = a[21];
out[11] = a[22];
out[12] = a[23];
out[13] = a[24];
out[14] = a[25];
out[15] = a[26];
out[16] = a[27];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
g1[3] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 90 */
static const int xBoundIndices[ 90 ] = 
{ 18, 19, 20, 30, 31, 32, 42, 43, 44, 54, 55, 56, 66, 67, 68, 78, 79, 80, 90, 91, 92, 102, 103, 104, 114, 115, 116, 126, 127, 128, 138, 139, 140, 150, 151, 152, 162, 163, 164, 174, 175, 176, 186, 187, 188, 198, 199, 200, 210, 211, 212, 222, 223, 224, 234, 235, 236, 246, 247, 248, 258, 259, 260, 270, 271, 272, 282, 283, 284, 294, 295, 296, 306, 307, 308, 318, 319, 320, 330, 331, 332, 342, 343, 344, 354, 355, 356, 366, 367, 368 };
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
acado_moveGxT( &(acadoWorkspace.evGx[ lRun1 * 144 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ lRun1 * 12-12 ]), &(acadoWorkspace.evGx[ lRun1 * 144 ]), &(acadoWorkspace.d[ lRun1 * 12 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ lRun1 * 144-144 ]), &(acadoWorkspace.evGx[ lRun1 * 144 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ lRun4 * 48 ]), &(acadoWorkspace.E[ lRun3 * 48 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun1 * 48 ]), &(acadoWorkspace.E[ lRun3 * 48 ]) );
}

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( &(acadoWorkspace.Q1[ lRun1 * 144 + 144 ]), &(acadoWorkspace.E[ lRun3 * 48 ]), &(acadoWorkspace.QE[ lRun3 * 48 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ lRun3 * 48 ]), &(acadoWorkspace.QE[ lRun3 * 48 ]) );
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_zeroBlockH10( &(acadoWorkspace.H10[ lRun1 * 48 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multQETGx( &(acadoWorkspace.QE[ lRun3 * 48 ]), &(acadoWorkspace.evGx[ lRun2 * 144 ]), &(acadoWorkspace.H10[ lRun1 * 48 ]) );
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acado_setBlockH11_R1( lRun1, lRun1, &(acadoWorkspace.R1[ lRun1 * 16 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 48 ]), &(acadoWorkspace.QE[ lRun5 * 48 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
acado_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
acado_setBlockH11( lRun1, lRun2, &(acadoWorkspace.E[ lRun4 * 48 ]), &(acadoWorkspace.QE[ lRun5 * 48 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun1, lRun2 );
}
}

acado_multQ1d( &(acadoWorkspace.Q1[ 144 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.Qd[ 12 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.Qd[ 24 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 720 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.Qd[ 48 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 864 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.Qd[ 60 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1008 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1152 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.Qd[ 84 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.Qd[ 96 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1440 ]), &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.Qd[ 108 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1584 ]), &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.Qd[ 120 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1728 ]), &(acadoWorkspace.d[ 132 ]), &(acadoWorkspace.Qd[ 132 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 1872 ]), &(acadoWorkspace.d[ 144 ]), &(acadoWorkspace.Qd[ 144 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2016 ]), &(acadoWorkspace.d[ 156 ]), &(acadoWorkspace.Qd[ 156 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2160 ]), &(acadoWorkspace.d[ 168 ]), &(acadoWorkspace.Qd[ 168 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2304 ]), &(acadoWorkspace.d[ 180 ]), &(acadoWorkspace.Qd[ 180 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2448 ]), &(acadoWorkspace.d[ 192 ]), &(acadoWorkspace.Qd[ 192 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.d[ 204 ]), &(acadoWorkspace.Qd[ 204 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2736 ]), &(acadoWorkspace.d[ 216 ]), &(acadoWorkspace.Qd[ 216 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 2880 ]), &(acadoWorkspace.d[ 228 ]), &(acadoWorkspace.Qd[ 228 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3024 ]), &(acadoWorkspace.d[ 240 ]), &(acadoWorkspace.Qd[ 240 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3168 ]), &(acadoWorkspace.d[ 252 ]), &(acadoWorkspace.Qd[ 252 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3312 ]), &(acadoWorkspace.d[ 264 ]), &(acadoWorkspace.Qd[ 264 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3456 ]), &(acadoWorkspace.d[ 276 ]), &(acadoWorkspace.Qd[ 276 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3600 ]), &(acadoWorkspace.d[ 288 ]), &(acadoWorkspace.Qd[ 288 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3744 ]), &(acadoWorkspace.d[ 300 ]), &(acadoWorkspace.Qd[ 300 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 3888 ]), &(acadoWorkspace.d[ 312 ]), &(acadoWorkspace.Qd[ 312 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4032 ]), &(acadoWorkspace.d[ 324 ]), &(acadoWorkspace.Qd[ 324 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 4176 ]), &(acadoWorkspace.d[ 336 ]), &(acadoWorkspace.Qd[ 336 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 348 ]), &(acadoWorkspace.Qd[ 348 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_macETSlu( &(acadoWorkspace.QE[ lRun3 * 48 ]), &(acadoWorkspace.g[ lRun1 * 4 ]) );
}
}
acadoWorkspace.lb[0] = - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[3];
acadoWorkspace.lb[4] = - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[7];
acadoWorkspace.lb[8] = - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[11];
acadoWorkspace.lb[12] = - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[15];
acadoWorkspace.lb[16] = - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[19];
acadoWorkspace.lb[20] = - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[23];
acadoWorkspace.lb[24] = - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[27];
acadoWorkspace.lb[28] = - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[31];
acadoWorkspace.lb[32] = - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[35];
acadoWorkspace.lb[36] = - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[39];
acadoWorkspace.lb[40] = - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[43];
acadoWorkspace.lb[44] = - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[47];
acadoWorkspace.lb[48] = - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[51];
acadoWorkspace.lb[52] = - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[55];
acadoWorkspace.lb[56] = - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[59];
acadoWorkspace.lb[60] = - acadoVariables.u[60];
acadoWorkspace.lb[61] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[61];
acadoWorkspace.lb[62] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[62];
acadoWorkspace.lb[63] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[63];
acadoWorkspace.lb[64] = - acadoVariables.u[64];
acadoWorkspace.lb[65] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[65];
acadoWorkspace.lb[66] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[66];
acadoWorkspace.lb[67] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[67];
acadoWorkspace.lb[68] = - acadoVariables.u[68];
acadoWorkspace.lb[69] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[69];
acadoWorkspace.lb[70] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[70];
acadoWorkspace.lb[71] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[71];
acadoWorkspace.lb[72] = - acadoVariables.u[72];
acadoWorkspace.lb[73] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[73];
acadoWorkspace.lb[74] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[74];
acadoWorkspace.lb[75] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[75];
acadoWorkspace.lb[76] = - acadoVariables.u[76];
acadoWorkspace.lb[77] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[77];
acadoWorkspace.lb[78] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[78];
acadoWorkspace.lb[79] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[79];
acadoWorkspace.lb[80] = - acadoVariables.u[80];
acadoWorkspace.lb[81] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[81];
acadoWorkspace.lb[82] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[82];
acadoWorkspace.lb[83] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[83];
acadoWorkspace.lb[84] = - acadoVariables.u[84];
acadoWorkspace.lb[85] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[85];
acadoWorkspace.lb[86] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[86];
acadoWorkspace.lb[87] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[87];
acadoWorkspace.lb[88] = - acadoVariables.u[88];
acadoWorkspace.lb[89] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[89];
acadoWorkspace.lb[90] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[90];
acadoWorkspace.lb[91] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[91];
acadoWorkspace.lb[92] = - acadoVariables.u[92];
acadoWorkspace.lb[93] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[93];
acadoWorkspace.lb[94] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[94];
acadoWorkspace.lb[95] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[95];
acadoWorkspace.lb[96] = - acadoVariables.u[96];
acadoWorkspace.lb[97] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[97];
acadoWorkspace.lb[98] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[98];
acadoWorkspace.lb[99] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[99];
acadoWorkspace.lb[100] = - acadoVariables.u[100];
acadoWorkspace.lb[101] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[101];
acadoWorkspace.lb[102] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[102];
acadoWorkspace.lb[103] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[103];
acadoWorkspace.lb[104] = - acadoVariables.u[104];
acadoWorkspace.lb[105] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[105];
acadoWorkspace.lb[106] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[106];
acadoWorkspace.lb[107] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[107];
acadoWorkspace.lb[108] = - acadoVariables.u[108];
acadoWorkspace.lb[109] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[109];
acadoWorkspace.lb[110] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[110];
acadoWorkspace.lb[111] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[111];
acadoWorkspace.lb[112] = - acadoVariables.u[112];
acadoWorkspace.lb[113] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[113];
acadoWorkspace.lb[114] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[114];
acadoWorkspace.lb[115] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[115];
acadoWorkspace.lb[116] = - acadoVariables.u[116];
acadoWorkspace.lb[117] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[117];
acadoWorkspace.lb[118] = (real_t)-8.4611250000000016e-03 - acadoVariables.u[118];
acadoWorkspace.lb[119] = (real_t)-9.0313011456628518e-03 - acadoVariables.u[119];
acadoWorkspace.ub[0] = (real_t)7.3575000000000002e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)8.4611250000000016e-03 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)8.4611250000000016e-03 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)9.0313011456628518e-03 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)7.3575000000000002e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)8.4611250000000016e-03 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)8.4611250000000016e-03 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)9.0313011456628518e-03 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)7.3575000000000002e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)8.4611250000000016e-03 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)8.4611250000000016e-03 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)9.0313011456628518e-03 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)7.3575000000000002e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)8.4611250000000016e-03 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)8.4611250000000016e-03 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)9.0313011456628518e-03 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)7.3575000000000002e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)8.4611250000000016e-03 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)8.4611250000000016e-03 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)9.0313011456628518e-03 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)7.3575000000000002e-01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)8.4611250000000016e-03 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)8.4611250000000016e-03 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)9.0313011456628518e-03 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)7.3575000000000002e-01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)8.4611250000000016e-03 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)8.4611250000000016e-03 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)9.0313011456628518e-03 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)7.3575000000000002e-01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)8.4611250000000016e-03 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)8.4611250000000016e-03 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)9.0313011456628518e-03 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)7.3575000000000002e-01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)8.4611250000000016e-03 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)8.4611250000000016e-03 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)9.0313011456628518e-03 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)7.3575000000000002e-01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)8.4611250000000016e-03 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)8.4611250000000016e-03 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)9.0313011456628518e-03 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)7.3575000000000002e-01 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)8.4611250000000016e-03 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)8.4611250000000016e-03 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)9.0313011456628518e-03 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)7.3575000000000002e-01 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)8.4611250000000016e-03 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)8.4611250000000016e-03 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)9.0313011456628518e-03 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)7.3575000000000002e-01 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)8.4611250000000016e-03 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)8.4611250000000016e-03 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)9.0313011456628518e-03 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)7.3575000000000002e-01 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)8.4611250000000016e-03 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)8.4611250000000016e-03 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)9.0313011456628518e-03 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)7.3575000000000002e-01 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)8.4611250000000016e-03 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)8.4611250000000016e-03 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)9.0313011456628518e-03 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)7.3575000000000002e-01 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)8.4611250000000016e-03 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)8.4611250000000016e-03 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)9.0313011456628518e-03 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)7.3575000000000002e-01 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)8.4611250000000016e-03 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)8.4611250000000016e-03 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)9.0313011456628518e-03 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)7.3575000000000002e-01 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)8.4611250000000016e-03 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)8.4611250000000016e-03 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)9.0313011456628518e-03 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)7.3575000000000002e-01 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)8.4611250000000016e-03 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)8.4611250000000016e-03 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)9.0313011456628518e-03 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)7.3575000000000002e-01 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)8.4611250000000016e-03 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)8.4611250000000016e-03 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)9.0313011456628518e-03 - acadoVariables.u[79];
acadoWorkspace.ub[80] = (real_t)7.3575000000000002e-01 - acadoVariables.u[80];
acadoWorkspace.ub[81] = (real_t)8.4611250000000016e-03 - acadoVariables.u[81];
acadoWorkspace.ub[82] = (real_t)8.4611250000000016e-03 - acadoVariables.u[82];
acadoWorkspace.ub[83] = (real_t)9.0313011456628518e-03 - acadoVariables.u[83];
acadoWorkspace.ub[84] = (real_t)7.3575000000000002e-01 - acadoVariables.u[84];
acadoWorkspace.ub[85] = (real_t)8.4611250000000016e-03 - acadoVariables.u[85];
acadoWorkspace.ub[86] = (real_t)8.4611250000000016e-03 - acadoVariables.u[86];
acadoWorkspace.ub[87] = (real_t)9.0313011456628518e-03 - acadoVariables.u[87];
acadoWorkspace.ub[88] = (real_t)7.3575000000000002e-01 - acadoVariables.u[88];
acadoWorkspace.ub[89] = (real_t)8.4611250000000016e-03 - acadoVariables.u[89];
acadoWorkspace.ub[90] = (real_t)8.4611250000000016e-03 - acadoVariables.u[90];
acadoWorkspace.ub[91] = (real_t)9.0313011456628518e-03 - acadoVariables.u[91];
acadoWorkspace.ub[92] = (real_t)7.3575000000000002e-01 - acadoVariables.u[92];
acadoWorkspace.ub[93] = (real_t)8.4611250000000016e-03 - acadoVariables.u[93];
acadoWorkspace.ub[94] = (real_t)8.4611250000000016e-03 - acadoVariables.u[94];
acadoWorkspace.ub[95] = (real_t)9.0313011456628518e-03 - acadoVariables.u[95];
acadoWorkspace.ub[96] = (real_t)7.3575000000000002e-01 - acadoVariables.u[96];
acadoWorkspace.ub[97] = (real_t)8.4611250000000016e-03 - acadoVariables.u[97];
acadoWorkspace.ub[98] = (real_t)8.4611250000000016e-03 - acadoVariables.u[98];
acadoWorkspace.ub[99] = (real_t)9.0313011456628518e-03 - acadoVariables.u[99];
acadoWorkspace.ub[100] = (real_t)7.3575000000000002e-01 - acadoVariables.u[100];
acadoWorkspace.ub[101] = (real_t)8.4611250000000016e-03 - acadoVariables.u[101];
acadoWorkspace.ub[102] = (real_t)8.4611250000000016e-03 - acadoVariables.u[102];
acadoWorkspace.ub[103] = (real_t)9.0313011456628518e-03 - acadoVariables.u[103];
acadoWorkspace.ub[104] = (real_t)7.3575000000000002e-01 - acadoVariables.u[104];
acadoWorkspace.ub[105] = (real_t)8.4611250000000016e-03 - acadoVariables.u[105];
acadoWorkspace.ub[106] = (real_t)8.4611250000000016e-03 - acadoVariables.u[106];
acadoWorkspace.ub[107] = (real_t)9.0313011456628518e-03 - acadoVariables.u[107];
acadoWorkspace.ub[108] = (real_t)7.3575000000000002e-01 - acadoVariables.u[108];
acadoWorkspace.ub[109] = (real_t)8.4611250000000016e-03 - acadoVariables.u[109];
acadoWorkspace.ub[110] = (real_t)8.4611250000000016e-03 - acadoVariables.u[110];
acadoWorkspace.ub[111] = (real_t)9.0313011456628518e-03 - acadoVariables.u[111];
acadoWorkspace.ub[112] = (real_t)7.3575000000000002e-01 - acadoVariables.u[112];
acadoWorkspace.ub[113] = (real_t)8.4611250000000016e-03 - acadoVariables.u[113];
acadoWorkspace.ub[114] = (real_t)8.4611250000000016e-03 - acadoVariables.u[114];
acadoWorkspace.ub[115] = (real_t)9.0313011456628518e-03 - acadoVariables.u[115];
acadoWorkspace.ub[116] = (real_t)7.3575000000000002e-01 - acadoVariables.u[116];
acadoWorkspace.ub[117] = (real_t)8.4611250000000016e-03 - acadoVariables.u[117];
acadoWorkspace.ub[118] = (real_t)8.4611250000000016e-03 - acadoVariables.u[118];
acadoWorkspace.ub[119] = (real_t)9.0313011456628518e-03 - acadoVariables.u[119];

for (lRun1 = 0; lRun1 < 90; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 12;
lRun4 = ((lRun3) / (12)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (12)) + ((lRun3) % (12));
acadoWorkspace.A[(lRun1 * 120) + (lRun2 * 4)] = acadoWorkspace.E[lRun5 * 4];
acadoWorkspace.A[(lRun1 * 120) + (lRun2 * 4 + 1)] = acadoWorkspace.E[lRun5 * 4 + 1];
acadoWorkspace.A[(lRun1 * 120) + (lRun2 * 4 + 2)] = acadoWorkspace.E[lRun5 * 4 + 2];
acadoWorkspace.A[(lRun1 * 120) + (lRun2 * 4 + 3)] = acadoWorkspace.E[lRun5 * 4 + 3];
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 12];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 12 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 12 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 12 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 12 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.x[lRun1 * 12 + 5];
acadoWorkspace.conValueIn[6] = acadoVariables.x[lRun1 * 12 + 6];
acadoWorkspace.conValueIn[7] = acadoVariables.x[lRun1 * 12 + 7];
acadoWorkspace.conValueIn[8] = acadoVariables.x[lRun1 * 12 + 8];
acadoWorkspace.conValueIn[9] = acadoVariables.x[lRun1 * 12 + 9];
acadoWorkspace.conValueIn[10] = acadoVariables.x[lRun1 * 12 + 10];
acadoWorkspace.conValueIn[11] = acadoVariables.x[lRun1 * 12 + 11];
acadoWorkspace.conValueIn[12] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.conValueIn[13] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[14] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[15] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.conValueIn[16] = acadoVariables.od[lRun1];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun1 * 12] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun1 * 12 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun1 * 12 + 2] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 12 + 3] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 12 + 4] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 12 + 5] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 12 + 6] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 12 + 7] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 12 + 8] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 12 + 9] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 12 + 10] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 12 + 11] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHu[lRun1 * 4] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHu[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHu[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHu[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[16];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];
acadoWorkspace.A01[6] = acadoWorkspace.evHx[6];
acadoWorkspace.A01[7] = acadoWorkspace.evHx[7];
acadoWorkspace.A01[8] = acadoWorkspace.evHx[8];
acadoWorkspace.A01[9] = acadoWorkspace.evHx[9];
acadoWorkspace.A01[10] = acadoWorkspace.evHx[10];
acadoWorkspace.A01[11] = acadoWorkspace.evHx[11];

acado_multHxC( &(acadoWorkspace.evHx[ 12 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 12 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.A01[ 24 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.A01[ 36 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.A01[ 48 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.A01[ 72 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.A01[ 84 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.A01[ 96 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.A01[ 108 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.A01[ 120 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.A01[ 132 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.A01[ 144 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.evGx[ 1728 ]), &(acadoWorkspace.A01[ 156 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.evGx[ 1872 ]), &(acadoWorkspace.A01[ 168 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.evGx[ 2016 ]), &(acadoWorkspace.A01[ 180 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.evGx[ 2160 ]), &(acadoWorkspace.A01[ 192 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.evGx[ 2304 ]), &(acadoWorkspace.A01[ 204 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.evGx[ 2448 ]), &(acadoWorkspace.A01[ 216 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.A01[ 228 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.evGx[ 2736 ]), &(acadoWorkspace.A01[ 240 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.evGx[ 2880 ]), &(acadoWorkspace.A01[ 252 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 264 ]), &(acadoWorkspace.evGx[ 3024 ]), &(acadoWorkspace.A01[ 264 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 276 ]), &(acadoWorkspace.evGx[ 3168 ]), &(acadoWorkspace.A01[ 276 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.evGx[ 3312 ]), &(acadoWorkspace.A01[ 288 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 300 ]), &(acadoWorkspace.evGx[ 3456 ]), &(acadoWorkspace.A01[ 300 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 312 ]), &(acadoWorkspace.evGx[ 3600 ]), &(acadoWorkspace.A01[ 312 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 324 ]), &(acadoWorkspace.evGx[ 3744 ]), &(acadoWorkspace.A01[ 324 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 336 ]), &(acadoWorkspace.evGx[ 3888 ]), &(acadoWorkspace.A01[ 336 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 348 ]), &(acadoWorkspace.evGx[ 4032 ]), &(acadoWorkspace.A01[ 348 ]) );

for (lRun2 = 0; lRun2 < 29; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun3);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 12 + 12 ]), &(acadoWorkspace.E[ lRun4 * 48 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[10800] = acadoWorkspace.evHu[0];
acadoWorkspace.A[10801] = acadoWorkspace.evHu[1];
acadoWorkspace.A[10802] = acadoWorkspace.evHu[2];
acadoWorkspace.A[10803] = acadoWorkspace.evHu[3];
acadoWorkspace.A[10924] = acadoWorkspace.evHu[4];
acadoWorkspace.A[10925] = acadoWorkspace.evHu[5];
acadoWorkspace.A[10926] = acadoWorkspace.evHu[6];
acadoWorkspace.A[10927] = acadoWorkspace.evHu[7];
acadoWorkspace.A[11048] = acadoWorkspace.evHu[8];
acadoWorkspace.A[11049] = acadoWorkspace.evHu[9];
acadoWorkspace.A[11050] = acadoWorkspace.evHu[10];
acadoWorkspace.A[11051] = acadoWorkspace.evHu[11];
acadoWorkspace.A[11172] = acadoWorkspace.evHu[12];
acadoWorkspace.A[11173] = acadoWorkspace.evHu[13];
acadoWorkspace.A[11174] = acadoWorkspace.evHu[14];
acadoWorkspace.A[11175] = acadoWorkspace.evHu[15];
acadoWorkspace.A[11296] = acadoWorkspace.evHu[16];
acadoWorkspace.A[11297] = acadoWorkspace.evHu[17];
acadoWorkspace.A[11298] = acadoWorkspace.evHu[18];
acadoWorkspace.A[11299] = acadoWorkspace.evHu[19];
acadoWorkspace.A[11420] = acadoWorkspace.evHu[20];
acadoWorkspace.A[11421] = acadoWorkspace.evHu[21];
acadoWorkspace.A[11422] = acadoWorkspace.evHu[22];
acadoWorkspace.A[11423] = acadoWorkspace.evHu[23];
acadoWorkspace.A[11544] = acadoWorkspace.evHu[24];
acadoWorkspace.A[11545] = acadoWorkspace.evHu[25];
acadoWorkspace.A[11546] = acadoWorkspace.evHu[26];
acadoWorkspace.A[11547] = acadoWorkspace.evHu[27];
acadoWorkspace.A[11668] = acadoWorkspace.evHu[28];
acadoWorkspace.A[11669] = acadoWorkspace.evHu[29];
acadoWorkspace.A[11670] = acadoWorkspace.evHu[30];
acadoWorkspace.A[11671] = acadoWorkspace.evHu[31];
acadoWorkspace.A[11792] = acadoWorkspace.evHu[32];
acadoWorkspace.A[11793] = acadoWorkspace.evHu[33];
acadoWorkspace.A[11794] = acadoWorkspace.evHu[34];
acadoWorkspace.A[11795] = acadoWorkspace.evHu[35];
acadoWorkspace.A[11916] = acadoWorkspace.evHu[36];
acadoWorkspace.A[11917] = acadoWorkspace.evHu[37];
acadoWorkspace.A[11918] = acadoWorkspace.evHu[38];
acadoWorkspace.A[11919] = acadoWorkspace.evHu[39];
acadoWorkspace.A[12040] = acadoWorkspace.evHu[40];
acadoWorkspace.A[12041] = acadoWorkspace.evHu[41];
acadoWorkspace.A[12042] = acadoWorkspace.evHu[42];
acadoWorkspace.A[12043] = acadoWorkspace.evHu[43];
acadoWorkspace.A[12164] = acadoWorkspace.evHu[44];
acadoWorkspace.A[12165] = acadoWorkspace.evHu[45];
acadoWorkspace.A[12166] = acadoWorkspace.evHu[46];
acadoWorkspace.A[12167] = acadoWorkspace.evHu[47];
acadoWorkspace.A[12288] = acadoWorkspace.evHu[48];
acadoWorkspace.A[12289] = acadoWorkspace.evHu[49];
acadoWorkspace.A[12290] = acadoWorkspace.evHu[50];
acadoWorkspace.A[12291] = acadoWorkspace.evHu[51];
acadoWorkspace.A[12412] = acadoWorkspace.evHu[52];
acadoWorkspace.A[12413] = acadoWorkspace.evHu[53];
acadoWorkspace.A[12414] = acadoWorkspace.evHu[54];
acadoWorkspace.A[12415] = acadoWorkspace.evHu[55];
acadoWorkspace.A[12536] = acadoWorkspace.evHu[56];
acadoWorkspace.A[12537] = acadoWorkspace.evHu[57];
acadoWorkspace.A[12538] = acadoWorkspace.evHu[58];
acadoWorkspace.A[12539] = acadoWorkspace.evHu[59];
acadoWorkspace.A[12660] = acadoWorkspace.evHu[60];
acadoWorkspace.A[12661] = acadoWorkspace.evHu[61];
acadoWorkspace.A[12662] = acadoWorkspace.evHu[62];
acadoWorkspace.A[12663] = acadoWorkspace.evHu[63];
acadoWorkspace.A[12784] = acadoWorkspace.evHu[64];
acadoWorkspace.A[12785] = acadoWorkspace.evHu[65];
acadoWorkspace.A[12786] = acadoWorkspace.evHu[66];
acadoWorkspace.A[12787] = acadoWorkspace.evHu[67];
acadoWorkspace.A[12908] = acadoWorkspace.evHu[68];
acadoWorkspace.A[12909] = acadoWorkspace.evHu[69];
acadoWorkspace.A[12910] = acadoWorkspace.evHu[70];
acadoWorkspace.A[12911] = acadoWorkspace.evHu[71];
acadoWorkspace.A[13032] = acadoWorkspace.evHu[72];
acadoWorkspace.A[13033] = acadoWorkspace.evHu[73];
acadoWorkspace.A[13034] = acadoWorkspace.evHu[74];
acadoWorkspace.A[13035] = acadoWorkspace.evHu[75];
acadoWorkspace.A[13156] = acadoWorkspace.evHu[76];
acadoWorkspace.A[13157] = acadoWorkspace.evHu[77];
acadoWorkspace.A[13158] = acadoWorkspace.evHu[78];
acadoWorkspace.A[13159] = acadoWorkspace.evHu[79];
acadoWorkspace.A[13280] = acadoWorkspace.evHu[80];
acadoWorkspace.A[13281] = acadoWorkspace.evHu[81];
acadoWorkspace.A[13282] = acadoWorkspace.evHu[82];
acadoWorkspace.A[13283] = acadoWorkspace.evHu[83];
acadoWorkspace.A[13404] = acadoWorkspace.evHu[84];
acadoWorkspace.A[13405] = acadoWorkspace.evHu[85];
acadoWorkspace.A[13406] = acadoWorkspace.evHu[86];
acadoWorkspace.A[13407] = acadoWorkspace.evHu[87];
acadoWorkspace.A[13528] = acadoWorkspace.evHu[88];
acadoWorkspace.A[13529] = acadoWorkspace.evHu[89];
acadoWorkspace.A[13530] = acadoWorkspace.evHu[90];
acadoWorkspace.A[13531] = acadoWorkspace.evHu[91];
acadoWorkspace.A[13652] = acadoWorkspace.evHu[92];
acadoWorkspace.A[13653] = acadoWorkspace.evHu[93];
acadoWorkspace.A[13654] = acadoWorkspace.evHu[94];
acadoWorkspace.A[13655] = acadoWorkspace.evHu[95];
acadoWorkspace.A[13776] = acadoWorkspace.evHu[96];
acadoWorkspace.A[13777] = acadoWorkspace.evHu[97];
acadoWorkspace.A[13778] = acadoWorkspace.evHu[98];
acadoWorkspace.A[13779] = acadoWorkspace.evHu[99];
acadoWorkspace.A[13900] = acadoWorkspace.evHu[100];
acadoWorkspace.A[13901] = acadoWorkspace.evHu[101];
acadoWorkspace.A[13902] = acadoWorkspace.evHu[102];
acadoWorkspace.A[13903] = acadoWorkspace.evHu[103];
acadoWorkspace.A[14024] = acadoWorkspace.evHu[104];
acadoWorkspace.A[14025] = acadoWorkspace.evHu[105];
acadoWorkspace.A[14026] = acadoWorkspace.evHu[106];
acadoWorkspace.A[14027] = acadoWorkspace.evHu[107];
acadoWorkspace.A[14148] = acadoWorkspace.evHu[108];
acadoWorkspace.A[14149] = acadoWorkspace.evHu[109];
acadoWorkspace.A[14150] = acadoWorkspace.evHu[110];
acadoWorkspace.A[14151] = acadoWorkspace.evHu[111];
acadoWorkspace.A[14272] = acadoWorkspace.evHu[112];
acadoWorkspace.A[14273] = acadoWorkspace.evHu[113];
acadoWorkspace.A[14274] = acadoWorkspace.evHu[114];
acadoWorkspace.A[14275] = acadoWorkspace.evHu[115];
acadoWorkspace.A[14396] = acadoWorkspace.evHu[116];
acadoWorkspace.A[14397] = acadoWorkspace.evHu[117];
acadoWorkspace.A[14398] = acadoWorkspace.evHu[118];
acadoWorkspace.A[14399] = acadoWorkspace.evHu[119];
acadoWorkspace.lbA[90] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[0];
acadoWorkspace.lbA[91] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[1];
acadoWorkspace.lbA[92] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[2];
acadoWorkspace.lbA[93] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[3];
acadoWorkspace.lbA[94] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[4];
acadoWorkspace.lbA[95] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[5];
acadoWorkspace.lbA[96] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[6];
acadoWorkspace.lbA[97] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[7];
acadoWorkspace.lbA[98] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[8];
acadoWorkspace.lbA[99] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[9];
acadoWorkspace.lbA[100] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[10];
acadoWorkspace.lbA[101] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[11];
acadoWorkspace.lbA[102] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[12];
acadoWorkspace.lbA[103] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[13];
acadoWorkspace.lbA[104] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[14];
acadoWorkspace.lbA[105] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[15];
acadoWorkspace.lbA[106] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[16];
acadoWorkspace.lbA[107] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[17];
acadoWorkspace.lbA[108] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[18];
acadoWorkspace.lbA[109] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[19];
acadoWorkspace.lbA[110] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[20];
acadoWorkspace.lbA[111] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[21];
acadoWorkspace.lbA[112] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[22];
acadoWorkspace.lbA[113] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[23];
acadoWorkspace.lbA[114] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[24];
acadoWorkspace.lbA[115] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[25];
acadoWorkspace.lbA[116] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[26];
acadoWorkspace.lbA[117] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[27];
acadoWorkspace.lbA[118] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[28];
acadoWorkspace.lbA[119] = (real_t)5.0000000000000000e-01 - acadoWorkspace.evH[29];

acadoWorkspace.ubA[90] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[91] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[92] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[93] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[94] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[95] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[96] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[97] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[98] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[99] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[100] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[101] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[102] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[103] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[104] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[105] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[106] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[107] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[108] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[109] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[110] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[111] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[112] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[113] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[114] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[115] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[116] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[117] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[118] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[119] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[29];

acado_macHxd( &(acadoWorkspace.evHx[ 12 ]), acadoWorkspace.d, &(acadoWorkspace.lbA[ 91 ]), &(acadoWorkspace.ubA[ 91 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.d[ 12 ]), &(acadoWorkspace.lbA[ 92 ]), &(acadoWorkspace.ubA[ 92 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.d[ 24 ]), &(acadoWorkspace.lbA[ 93 ]), &(acadoWorkspace.ubA[ 93 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.lbA[ 94 ]), &(acadoWorkspace.ubA[ 94 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.d[ 48 ]), &(acadoWorkspace.lbA[ 95 ]), &(acadoWorkspace.ubA[ 95 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.d[ 60 ]), &(acadoWorkspace.lbA[ 96 ]), &(acadoWorkspace.ubA[ 96 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.lbA[ 97 ]), &(acadoWorkspace.ubA[ 97 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.d[ 84 ]), &(acadoWorkspace.lbA[ 98 ]), &(acadoWorkspace.ubA[ 98 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.d[ 96 ]), &(acadoWorkspace.lbA[ 99 ]), &(acadoWorkspace.ubA[ 99 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.d[ 108 ]), &(acadoWorkspace.lbA[ 100 ]), &(acadoWorkspace.ubA[ 100 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.d[ 120 ]), &(acadoWorkspace.lbA[ 101 ]), &(acadoWorkspace.ubA[ 101 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.d[ 132 ]), &(acadoWorkspace.lbA[ 102 ]), &(acadoWorkspace.ubA[ 102 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.d[ 144 ]), &(acadoWorkspace.lbA[ 103 ]), &(acadoWorkspace.ubA[ 103 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.d[ 156 ]), &(acadoWorkspace.lbA[ 104 ]), &(acadoWorkspace.ubA[ 104 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.d[ 168 ]), &(acadoWorkspace.lbA[ 105 ]), &(acadoWorkspace.ubA[ 105 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.d[ 180 ]), &(acadoWorkspace.lbA[ 106 ]), &(acadoWorkspace.ubA[ 106 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.d[ 192 ]), &(acadoWorkspace.lbA[ 107 ]), &(acadoWorkspace.ubA[ 107 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.d[ 204 ]), &(acadoWorkspace.lbA[ 108 ]), &(acadoWorkspace.ubA[ 108 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.d[ 216 ]), &(acadoWorkspace.lbA[ 109 ]), &(acadoWorkspace.ubA[ 109 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.d[ 228 ]), &(acadoWorkspace.lbA[ 110 ]), &(acadoWorkspace.ubA[ 110 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.d[ 240 ]), &(acadoWorkspace.lbA[ 111 ]), &(acadoWorkspace.ubA[ 111 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 264 ]), &(acadoWorkspace.d[ 252 ]), &(acadoWorkspace.lbA[ 112 ]), &(acadoWorkspace.ubA[ 112 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 276 ]), &(acadoWorkspace.d[ 264 ]), &(acadoWorkspace.lbA[ 113 ]), &(acadoWorkspace.ubA[ 113 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.d[ 276 ]), &(acadoWorkspace.lbA[ 114 ]), &(acadoWorkspace.ubA[ 114 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 300 ]), &(acadoWorkspace.d[ 288 ]), &(acadoWorkspace.lbA[ 115 ]), &(acadoWorkspace.ubA[ 115 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 312 ]), &(acadoWorkspace.d[ 300 ]), &(acadoWorkspace.lbA[ 116 ]), &(acadoWorkspace.ubA[ 116 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 324 ]), &(acadoWorkspace.d[ 312 ]), &(acadoWorkspace.lbA[ 117 ]), &(acadoWorkspace.ubA[ 117 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 336 ]), &(acadoWorkspace.d[ 324 ]), &(acadoWorkspace.lbA[ 118 ]), &(acadoWorkspace.ubA[ 118 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 348 ]), &(acadoWorkspace.d[ 336 ]), &(acadoWorkspace.lbA[ 119 ]), &(acadoWorkspace.ubA[ 119 ]) );

}

void acado_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];
acadoWorkspace.Dx0[10] = acadoVariables.x0[10] - acadoVariables.x[10];
acadoWorkspace.Dx0[11] = acadoVariables.x0[11] - acadoVariables.x[11];

for (lRun2 = 0; lRun2 < 300; ++lRun2)
acadoWorkspace.Dy[lRun2] -= acadoVariables.y[lRun2];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 40 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 80 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 160 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 200 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 280 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 320 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 360 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 400 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 440 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 480 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 520 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 560 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 600 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 640 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 680 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 720 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 760 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.g[ 76 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 800 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.g[ 80 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 840 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.g[ 84 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 880 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.g[ 88 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 920 ]), &(acadoWorkspace.Dy[ 230 ]), &(acadoWorkspace.g[ 92 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 960 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 96 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1000 ]), &(acadoWorkspace.Dy[ 250 ]), &(acadoWorkspace.g[ 100 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1040 ]), &(acadoWorkspace.Dy[ 260 ]), &(acadoWorkspace.g[ 104 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1080 ]), &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.g[ 108 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1120 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.g[ 112 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1160 ]), &(acadoWorkspace.Dy[ 290 ]), &(acadoWorkspace.g[ 116 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 120 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 360 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 480 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 600 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 840 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 960 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1080 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1200 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1320 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.QDy[ 132 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1440 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1560 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.QDy[ 156 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1680 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 168 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1800 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.QDy[ 180 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1920 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.QDy[ 192 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2040 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.QDy[ 204 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2160 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 216 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2280 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.QDy[ 228 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2400 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.QDy[ 240 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2520 ]), &(acadoWorkspace.Dy[ 210 ]), &(acadoWorkspace.QDy[ 252 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2640 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.QDy[ 264 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2760 ]), &(acadoWorkspace.Dy[ 230 ]), &(acadoWorkspace.QDy[ 276 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2880 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 288 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3000 ]), &(acadoWorkspace.Dy[ 250 ]), &(acadoWorkspace.QDy[ 300 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3120 ]), &(acadoWorkspace.Dy[ 260 ]), &(acadoWorkspace.QDy[ 312 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3240 ]), &(acadoWorkspace.Dy[ 270 ]), &(acadoWorkspace.QDy[ 324 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3360 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.QDy[ 336 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3480 ]), &(acadoWorkspace.Dy[ 290 ]), &(acadoWorkspace.QDy[ 348 ]) );

acadoWorkspace.QDy[360] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[361] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[362] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[363] = + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[364] = + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[365] = + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[366] = + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[367] = + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[368] = + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[369] = + acadoWorkspace.QN2[54]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[55]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[56]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[57]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[58]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[59]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[370] = + acadoWorkspace.QN2[60]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[61]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[62]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[63]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[64]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[65]*acadoWorkspace.DyN[5];
acadoWorkspace.QDy[371] = + acadoWorkspace.QN2[66]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[67]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[68]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[69]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[70]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[71]*acadoWorkspace.DyN[5];

for (lRun2 = 0; lRun2 < 360; ++lRun2)
acadoWorkspace.QDy[lRun2 + 12] += acadoWorkspace.Qd[lRun2];


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
acado_multEQDy( &(acadoWorkspace.E[ lRun3 * 48 ]), &(acadoWorkspace.QDy[ lRun2 * 12 + 12 ]), &(acadoWorkspace.g[ lRun1 * 4 ]) );
}
}

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[1] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[2] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[3] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[4] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[5] += + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[6] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[7] += + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[8] += + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[9] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[10] += + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[11] += + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[12] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[13] += + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[14] += + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[15] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[16] += + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[17] += + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[18] += + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[19] += + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[20] += + acadoWorkspace.H10[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[245]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[246]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[247]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[248]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[249]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[250]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[251]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[21] += + acadoWorkspace.H10[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[256]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[257]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[258]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[259]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[260]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[261]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[262]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[263]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[22] += + acadoWorkspace.H10[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[268]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[269]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[270]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[271]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[272]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[273]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[274]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[275]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[23] += + acadoWorkspace.H10[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[280]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[281]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[282]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[283]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[284]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[285]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[286]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[287]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[24] += + acadoWorkspace.H10[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[292]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[293]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[294]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[295]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[296]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[297]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[298]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[299]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[25] += + acadoWorkspace.H10[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[305]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[306]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[307]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[308]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[309]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[310]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[311]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[26] += + acadoWorkspace.H10[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[316]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[317]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[318]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[319]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[320]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[321]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[322]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[323]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[27] += + acadoWorkspace.H10[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[327]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[328]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[329]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[330]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[331]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[332]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[333]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[334]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[335]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[28] += + acadoWorkspace.H10[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[340]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[341]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[342]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[343]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[344]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[345]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[346]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[347]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[29] += + acadoWorkspace.H10[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[351]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[352]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[353]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[354]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[355]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[356]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[357]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[358]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[359]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[30] += + acadoWorkspace.H10[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[365]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[366]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[367]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[368]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[369]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[370]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[371]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[31] += + acadoWorkspace.H10[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[375]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[376]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[377]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[378]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[379]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[380]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[381]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[382]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[383]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[32] += + acadoWorkspace.H10[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[387]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[388]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[389]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[390]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[391]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[392]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[393]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[394]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[395]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[33] += + acadoWorkspace.H10[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[399]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[400]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[401]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[402]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[403]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[404]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[405]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[406]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[407]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[34] += + acadoWorkspace.H10[408]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[409]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[410]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[411]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[412]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[413]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[414]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[415]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[416]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[417]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[418]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[419]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[35] += + acadoWorkspace.H10[420]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[421]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[422]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[423]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[424]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[425]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[426]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[427]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[428]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[429]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[430]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[431]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[36] += + acadoWorkspace.H10[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[435]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[436]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[437]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[438]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[439]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[440]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[441]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[442]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[443]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[37] += + acadoWorkspace.H10[444]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[445]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[446]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[447]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[448]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[449]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[450]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[451]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[452]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[453]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[454]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[455]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[38] += + acadoWorkspace.H10[456]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[457]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[458]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[459]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[460]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[461]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[462]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[463]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[464]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[465]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[466]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[467]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[39] += + acadoWorkspace.H10[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[471]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[472]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[473]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[474]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[475]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[476]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[477]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[478]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[479]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[40] += + acadoWorkspace.H10[480]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[481]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[482]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[483]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[484]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[485]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[486]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[487]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[488]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[489]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[490]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[491]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[41] += + acadoWorkspace.H10[492]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[493]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[494]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[495]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[496]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[497]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[498]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[499]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[500]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[501]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[502]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[503]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[42] += + acadoWorkspace.H10[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[507]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[508]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[509]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[510]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[511]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[512]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[513]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[514]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[515]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[43] += + acadoWorkspace.H10[516]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[517]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[518]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[519]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[520]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[521]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[522]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[523]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[524]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[525]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[526]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[527]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[44] += + acadoWorkspace.H10[528]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[529]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[530]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[531]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[532]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[533]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[534]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[535]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[536]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[537]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[538]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[539]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[45] += + acadoWorkspace.H10[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[544]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[545]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[546]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[547]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[548]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[549]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[550]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[551]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[46] += + acadoWorkspace.H10[552]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[553]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[554]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[555]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[556]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[557]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[558]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[559]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[560]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[561]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[562]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[563]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[47] += + acadoWorkspace.H10[564]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[565]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[566]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[567]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[568]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[569]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[570]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[571]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[572]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[573]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[574]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[575]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[48] += + acadoWorkspace.H10[576]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[577]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[578]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[579]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[580]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[581]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[582]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[583]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[584]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[585]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[586]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[587]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[49] += + acadoWorkspace.H10[588]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[589]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[590]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[591]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[592]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[593]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[594]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[595]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[596]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[597]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[598]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[599]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[50] += + acadoWorkspace.H10[600]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[601]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[602]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[603]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[604]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[605]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[606]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[607]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[608]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[609]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[610]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[611]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[51] += + acadoWorkspace.H10[612]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[613]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[614]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[615]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[616]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[617]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[618]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[619]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[620]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[621]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[622]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[623]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[52] += + acadoWorkspace.H10[624]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[625]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[626]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[627]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[628]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[629]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[630]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[631]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[632]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[633]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[634]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[635]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[53] += + acadoWorkspace.H10[636]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[637]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[638]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[639]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[640]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[641]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[642]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[643]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[644]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[645]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[646]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[647]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[54] += + acadoWorkspace.H10[648]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[649]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[650]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[651]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[652]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[653]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[654]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[655]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[656]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[657]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[658]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[659]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[55] += + acadoWorkspace.H10[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[662]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[663]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[664]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[665]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[666]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[667]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[668]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[669]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[670]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[671]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[56] += + acadoWorkspace.H10[672]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[673]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[674]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[675]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[676]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[677]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[678]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[679]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[680]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[681]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[682]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[683]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[57] += + acadoWorkspace.H10[684]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[685]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[686]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[687]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[688]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[689]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[690]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[691]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[692]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[693]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[694]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[695]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[58] += + acadoWorkspace.H10[696]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[697]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[698]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[699]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[700]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[701]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[702]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[703]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[704]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[705]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[706]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[707]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[59] += + acadoWorkspace.H10[708]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[709]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[710]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[711]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[712]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[713]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[714]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[715]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[716]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[717]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[718]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[719]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[60] += + acadoWorkspace.H10[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[723]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[724]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[725]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[726]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[727]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[728]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[729]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[730]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[731]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[61] += + acadoWorkspace.H10[732]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[733]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[734]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[735]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[736]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[737]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[738]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[739]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[740]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[741]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[742]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[743]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[62] += + acadoWorkspace.H10[744]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[745]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[746]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[747]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[748]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[749]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[750]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[751]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[752]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[753]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[754]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[755]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[63] += + acadoWorkspace.H10[756]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[757]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[758]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[759]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[760]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[761]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[762]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[763]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[764]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[765]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[766]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[767]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[64] += + acadoWorkspace.H10[768]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[769]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[770]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[771]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[772]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[773]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[774]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[775]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[776]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[777]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[778]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[779]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[65] += + acadoWorkspace.H10[780]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[781]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[782]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[783]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[784]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[785]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[786]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[787]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[788]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[789]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[790]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[791]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[66] += + acadoWorkspace.H10[792]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[793]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[794]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[795]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[796]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[797]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[798]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[799]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[800]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[801]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[802]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[803]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[67] += + acadoWorkspace.H10[804]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[805]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[806]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[807]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[808]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[809]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[810]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[811]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[812]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[813]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[814]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[815]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[68] += + acadoWorkspace.H10[816]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[817]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[818]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[819]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[820]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[821]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[822]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[823]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[824]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[825]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[826]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[827]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[69] += + acadoWorkspace.H10[828]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[829]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[830]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[831]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[832]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[833]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[834]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[835]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[836]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[837]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[838]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[839]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[70] += + acadoWorkspace.H10[840]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[841]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[842]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[843]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[844]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[845]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[846]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[847]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[848]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[849]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[850]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[851]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[71] += + acadoWorkspace.H10[852]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[853]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[854]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[855]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[856]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[857]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[858]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[859]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[860]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[861]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[862]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[863]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[72] += + acadoWorkspace.H10[864]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[865]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[866]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[867]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[868]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[869]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[870]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[871]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[872]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[873]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[874]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[875]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[73] += + acadoWorkspace.H10[876]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[877]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[878]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[879]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[880]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[881]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[882]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[883]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[884]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[885]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[886]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[887]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[74] += + acadoWorkspace.H10[888]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[889]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[890]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[891]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[892]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[893]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[894]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[895]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[896]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[897]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[898]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[899]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[75] += + acadoWorkspace.H10[900]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[901]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[902]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[903]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[904]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[905]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[906]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[907]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[908]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[909]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[910]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[911]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[76] += + acadoWorkspace.H10[912]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[913]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[914]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[915]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[916]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[917]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[918]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[919]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[920]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[921]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[922]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[923]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[77] += + acadoWorkspace.H10[924]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[925]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[926]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[927]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[928]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[929]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[930]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[931]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[932]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[933]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[934]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[935]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[78] += + acadoWorkspace.H10[936]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[937]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[938]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[939]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[940]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[941]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[942]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[943]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[944]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[945]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[946]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[947]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[79] += + acadoWorkspace.H10[948]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[949]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[950]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[951]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[952]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[953]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[954]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[955]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[956]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[957]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[958]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[959]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[80] += + acadoWorkspace.H10[960]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[961]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[962]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[963]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[964]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[965]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[966]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[967]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[968]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[969]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[970]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[971]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[81] += + acadoWorkspace.H10[972]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[973]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[974]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[975]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[976]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[977]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[978]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[979]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[980]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[981]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[982]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[983]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[82] += + acadoWorkspace.H10[984]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[985]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[986]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[987]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[988]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[989]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[990]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[991]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[992]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[993]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[994]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[995]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[83] += + acadoWorkspace.H10[996]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[997]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[998]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[999]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1000]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1001]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1002]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1003]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1004]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1005]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1006]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1007]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[84] += + acadoWorkspace.H10[1008]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1009]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1010]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1011]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1012]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1013]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1014]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1015]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1016]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1017]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1018]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1019]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[85] += + acadoWorkspace.H10[1020]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1021]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1022]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1023]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1024]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1025]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1026]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1027]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1028]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1029]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1030]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1031]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[86] += + acadoWorkspace.H10[1032]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1033]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1034]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1035]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1036]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1037]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1038]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1039]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1040]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1041]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1042]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1043]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[87] += + acadoWorkspace.H10[1044]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1045]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1046]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1047]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1048]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1049]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1050]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1051]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1052]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1053]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1054]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1055]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[88] += + acadoWorkspace.H10[1056]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1057]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1058]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1059]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1060]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1061]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1062]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1063]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1064]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1065]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1066]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1067]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[89] += + acadoWorkspace.H10[1068]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1069]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1070]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1071]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1072]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1073]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1074]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1075]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1076]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1077]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1078]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1079]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[90] += + acadoWorkspace.H10[1080]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1081]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1082]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1083]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1084]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1085]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1086]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1087]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1088]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1089]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1090]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1091]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[91] += + acadoWorkspace.H10[1092]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1093]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1094]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1095]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1096]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1097]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1098]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1099]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1100]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1101]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1102]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1103]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[92] += + acadoWorkspace.H10[1104]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1105]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1106]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1107]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1108]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1109]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1110]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1111]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1112]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1113]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1114]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1115]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[93] += + acadoWorkspace.H10[1116]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1117]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1118]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1119]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1120]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1121]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1122]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1123]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1124]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1125]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1126]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1127]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[94] += + acadoWorkspace.H10[1128]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1129]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1130]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1131]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1132]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1133]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1134]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1135]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1136]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1137]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1138]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1139]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[95] += + acadoWorkspace.H10[1140]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1141]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1142]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1143]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1144]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1145]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1146]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1147]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1148]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1149]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1150]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1151]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[96] += + acadoWorkspace.H10[1152]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1153]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1154]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1155]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1156]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1157]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1158]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1159]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1160]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1161]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1162]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1163]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[97] += + acadoWorkspace.H10[1164]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1165]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1166]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1167]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1168]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1169]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1170]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1171]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1172]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1173]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1174]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1175]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[98] += + acadoWorkspace.H10[1176]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1177]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1178]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1179]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1180]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1181]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1182]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1183]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1184]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1185]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1186]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1187]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[99] += + acadoWorkspace.H10[1188]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1189]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1190]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1191]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1192]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1193]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1194]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1195]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1196]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1197]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1198]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1199]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[100] += + acadoWorkspace.H10[1200]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1201]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1202]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1203]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1204]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1205]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1206]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1207]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1208]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1209]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1210]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1211]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[101] += + acadoWorkspace.H10[1212]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1213]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1214]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1215]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1216]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1217]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1218]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1219]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1220]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1221]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1222]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1223]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[102] += + acadoWorkspace.H10[1224]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1225]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1226]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1227]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1228]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1229]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1230]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1231]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1232]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1233]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1234]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1235]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[103] += + acadoWorkspace.H10[1236]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1237]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1238]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1239]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1240]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1241]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1242]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1243]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1244]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1245]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1246]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1247]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[104] += + acadoWorkspace.H10[1248]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1249]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1250]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1251]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1252]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1253]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1254]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1255]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1256]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1257]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1258]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1259]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[105] += + acadoWorkspace.H10[1260]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1261]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1262]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1263]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1264]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1265]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1266]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1267]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1268]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1269]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1270]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1271]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[106] += + acadoWorkspace.H10[1272]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1273]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1274]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1275]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1276]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1277]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1278]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1279]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1280]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1281]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1282]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1283]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[107] += + acadoWorkspace.H10[1284]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1285]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1286]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1287]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1288]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1289]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1290]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1291]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1292]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1293]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1294]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1295]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[108] += + acadoWorkspace.H10[1296]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1297]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1298]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1299]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1300]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1301]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1302]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1303]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1304]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1305]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1306]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1307]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[109] += + acadoWorkspace.H10[1308]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1309]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1310]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1311]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1312]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1313]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1314]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1315]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1316]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1317]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1318]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1319]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[110] += + acadoWorkspace.H10[1320]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1321]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1322]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1323]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1324]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1325]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1326]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1327]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1328]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1329]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1330]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1331]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[111] += + acadoWorkspace.H10[1332]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1333]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1334]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1335]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1336]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1337]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1338]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1339]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1340]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1341]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1342]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1343]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[112] += + acadoWorkspace.H10[1344]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1345]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1346]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1347]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1348]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1349]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1350]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1351]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1352]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1353]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1354]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1355]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[113] += + acadoWorkspace.H10[1356]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1357]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1358]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1359]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1360]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1361]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1362]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1363]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1364]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1365]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1366]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1367]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[114] += + acadoWorkspace.H10[1368]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1369]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1370]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1371]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1372]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1373]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1374]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1375]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1376]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1377]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1378]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1379]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[115] += + acadoWorkspace.H10[1380]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1381]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1382]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1383]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1384]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1385]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1386]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1387]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1388]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1389]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1390]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1391]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[116] += + acadoWorkspace.H10[1392]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1393]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1394]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1395]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1396]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1397]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1398]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1399]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1400]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1401]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1402]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1403]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[117] += + acadoWorkspace.H10[1404]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1405]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1406]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1407]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1408]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1409]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1410]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1411]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1412]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1413]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1414]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1415]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[118] += + acadoWorkspace.H10[1416]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1417]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1418]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1419]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1420]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1421]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1422]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1423]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1424]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1425]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1426]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1427]*acadoWorkspace.Dx0[11];
acadoWorkspace.g[119] += + acadoWorkspace.H10[1428]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1429]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[1430]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[1431]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[1432]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[1433]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[1434]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[1435]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[1436]*acadoWorkspace.Dx0[8] + acadoWorkspace.H10[1437]*acadoWorkspace.Dx0[9] + acadoWorkspace.H10[1438]*acadoWorkspace.Dx0[10] + acadoWorkspace.H10[1439]*acadoWorkspace.Dx0[11];

tmp = + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[11] + acadoVariables.x[18];
tmp += acadoWorkspace.d[6];
acadoWorkspace.lbA[0] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[0] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[11] + acadoVariables.x[19];
tmp += acadoWorkspace.d[7];
acadoWorkspace.lbA[1] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[1] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[11] + acadoVariables.x[20];
tmp += acadoWorkspace.d[8];
acadoWorkspace.lbA[2] = - tmp;
acadoWorkspace.ubA[2] = - tmp;
tmp = + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[11] + acadoVariables.x[30];
tmp += acadoWorkspace.d[18];
acadoWorkspace.lbA[3] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[3] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[11] + acadoVariables.x[31];
tmp += acadoWorkspace.d[19];
acadoWorkspace.lbA[4] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[4] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[11] + acadoVariables.x[32];
tmp += acadoWorkspace.d[20];
acadoWorkspace.lbA[5] = - tmp;
acadoWorkspace.ubA[5] = - tmp;
tmp = + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[11] + acadoVariables.x[42];
tmp += acadoWorkspace.d[30];
acadoWorkspace.lbA[6] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[6] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[11] + acadoVariables.x[43];
tmp += acadoWorkspace.d[31];
acadoWorkspace.lbA[7] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[7] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[11] + acadoVariables.x[44];
tmp += acadoWorkspace.d[32];
acadoWorkspace.lbA[8] = - tmp;
acadoWorkspace.ubA[8] = - tmp;
tmp = + acadoWorkspace.evGx[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[507]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[508]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[509]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[510]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[511]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[512]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[513]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[514]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[11] + acadoVariables.x[54];
tmp += acadoWorkspace.d[42];
acadoWorkspace.lbA[9] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[9] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[525]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[526]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[527]*acadoWorkspace.Dx0[11] + acadoVariables.x[55];
tmp += acadoWorkspace.d[43];
acadoWorkspace.lbA[10] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[10] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[528]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[529]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[530]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[531]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[532]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[533]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[534]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[535]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[536]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[537]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[538]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[539]*acadoWorkspace.Dx0[11] + acadoVariables.x[56];
tmp += acadoWorkspace.d[44];
acadoWorkspace.lbA[11] = - tmp;
acadoWorkspace.ubA[11] = - tmp;
tmp = + acadoWorkspace.evGx[648]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[649]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[650]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[651]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[652]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[653]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[654]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[655]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[656]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[657]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[658]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[659]*acadoWorkspace.Dx0[11] + acadoVariables.x[66];
tmp += acadoWorkspace.d[54];
acadoWorkspace.lbA[12] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[12] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[660]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[661]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[662]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[663]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[664]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[665]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[666]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[667]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[668]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[669]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[670]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[671]*acadoWorkspace.Dx0[11] + acadoVariables.x[67];
tmp += acadoWorkspace.d[55];
acadoWorkspace.lbA[13] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[13] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[672]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[673]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[674]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[675]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[676]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[677]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[678]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[679]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[680]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[681]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[682]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[683]*acadoWorkspace.Dx0[11] + acadoVariables.x[68];
tmp += acadoWorkspace.d[56];
acadoWorkspace.lbA[14] = - tmp;
acadoWorkspace.ubA[14] = - tmp;
tmp = + acadoWorkspace.evGx[792]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[793]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[794]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[795]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[796]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[797]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[798]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[799]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[800]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[801]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[802]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[803]*acadoWorkspace.Dx0[11] + acadoVariables.x[78];
tmp += acadoWorkspace.d[66];
acadoWorkspace.lbA[15] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[15] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[804]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[805]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[806]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[807]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[808]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[809]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[810]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[811]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[812]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[813]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[814]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[815]*acadoWorkspace.Dx0[11] + acadoVariables.x[79];
tmp += acadoWorkspace.d[67];
acadoWorkspace.lbA[16] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[16] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[816]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[817]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[818]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[819]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[820]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[821]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[822]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[823]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[824]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[825]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[826]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[827]*acadoWorkspace.Dx0[11] + acadoVariables.x[80];
tmp += acadoWorkspace.d[68];
acadoWorkspace.lbA[17] = - tmp;
acadoWorkspace.ubA[17] = - tmp;
tmp = + acadoWorkspace.evGx[936]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[937]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[938]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[939]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[940]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[941]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[942]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[943]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[944]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[945]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[946]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[947]*acadoWorkspace.Dx0[11] + acadoVariables.x[90];
tmp += acadoWorkspace.d[78];
acadoWorkspace.lbA[18] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[18] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[948]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[949]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[950]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[951]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[952]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[953]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[954]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[955]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[956]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[957]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[958]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[959]*acadoWorkspace.Dx0[11] + acadoVariables.x[91];
tmp += acadoWorkspace.d[79];
acadoWorkspace.lbA[19] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[19] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[960]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[961]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[962]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[963]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[964]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[965]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[966]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[967]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[968]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[969]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[970]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[971]*acadoWorkspace.Dx0[11] + acadoVariables.x[92];
tmp += acadoWorkspace.d[80];
acadoWorkspace.lbA[20] = - tmp;
acadoWorkspace.ubA[20] = - tmp;
tmp = + acadoWorkspace.evGx[1080]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1081]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1082]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1083]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1084]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1085]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1086]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1087]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1088]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1089]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1090]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1091]*acadoWorkspace.Dx0[11] + acadoVariables.x[102];
tmp += acadoWorkspace.d[90];
acadoWorkspace.lbA[21] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[21] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1092]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1093]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1094]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1095]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1096]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1097]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1098]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1099]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1100]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1101]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1102]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1103]*acadoWorkspace.Dx0[11] + acadoVariables.x[103];
tmp += acadoWorkspace.d[91];
acadoWorkspace.lbA[22] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[22] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1104]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1105]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1106]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1107]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1108]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1109]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1110]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1111]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1112]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1113]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1114]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1115]*acadoWorkspace.Dx0[11] + acadoVariables.x[104];
tmp += acadoWorkspace.d[92];
acadoWorkspace.lbA[23] = - tmp;
acadoWorkspace.ubA[23] = - tmp;
tmp = + acadoWorkspace.evGx[1224]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1225]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1226]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1227]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1228]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1229]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1230]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1231]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1232]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1233]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1234]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1235]*acadoWorkspace.Dx0[11] + acadoVariables.x[114];
tmp += acadoWorkspace.d[102];
acadoWorkspace.lbA[24] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[24] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1236]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1237]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1238]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1239]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1240]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1241]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1242]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1243]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1244]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1245]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1246]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1247]*acadoWorkspace.Dx0[11] + acadoVariables.x[115];
tmp += acadoWorkspace.d[103];
acadoWorkspace.lbA[25] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[25] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1248]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1249]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1250]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1251]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1252]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1253]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1254]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1255]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1256]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1257]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1258]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1259]*acadoWorkspace.Dx0[11] + acadoVariables.x[116];
tmp += acadoWorkspace.d[104];
acadoWorkspace.lbA[26] = - tmp;
acadoWorkspace.ubA[26] = - tmp;
tmp = + acadoWorkspace.evGx[1368]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1369]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1370]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1371]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1372]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1373]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1374]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1375]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1376]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1377]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1378]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1379]*acadoWorkspace.Dx0[11] + acadoVariables.x[126];
tmp += acadoWorkspace.d[114];
acadoWorkspace.lbA[27] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[27] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1380]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1381]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1382]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1383]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1384]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1385]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1386]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1387]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1388]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1389]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1390]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1391]*acadoWorkspace.Dx0[11] + acadoVariables.x[127];
tmp += acadoWorkspace.d[115];
acadoWorkspace.lbA[28] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[28] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1392]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1393]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1394]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1395]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1396]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1397]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1398]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1399]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1400]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1401]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1402]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1403]*acadoWorkspace.Dx0[11] + acadoVariables.x[128];
tmp += acadoWorkspace.d[116];
acadoWorkspace.lbA[29] = - tmp;
acadoWorkspace.ubA[29] = - tmp;
tmp = + acadoWorkspace.evGx[1512]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1513]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1514]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1515]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1516]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1517]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1518]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1519]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1520]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1521]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1522]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1523]*acadoWorkspace.Dx0[11] + acadoVariables.x[138];
tmp += acadoWorkspace.d[126];
acadoWorkspace.lbA[30] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[30] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1524]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1525]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1526]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1527]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1528]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1529]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1530]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1531]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1532]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1533]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1534]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1535]*acadoWorkspace.Dx0[11] + acadoVariables.x[139];
tmp += acadoWorkspace.d[127];
acadoWorkspace.lbA[31] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[31] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1536]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1537]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1538]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1539]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1540]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1541]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1542]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1543]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1544]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1545]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1546]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1547]*acadoWorkspace.Dx0[11] + acadoVariables.x[140];
tmp += acadoWorkspace.d[128];
acadoWorkspace.lbA[32] = - tmp;
acadoWorkspace.ubA[32] = - tmp;
tmp = + acadoWorkspace.evGx[1656]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1657]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1658]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1659]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1660]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1661]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1662]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1663]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1664]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1665]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1666]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1667]*acadoWorkspace.Dx0[11] + acadoVariables.x[150];
tmp += acadoWorkspace.d[138];
acadoWorkspace.lbA[33] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[33] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1668]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1669]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1670]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1671]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1672]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1673]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1674]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1675]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1676]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1677]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1678]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1679]*acadoWorkspace.Dx0[11] + acadoVariables.x[151];
tmp += acadoWorkspace.d[139];
acadoWorkspace.lbA[34] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[34] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1680]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1681]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1682]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1683]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1684]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1685]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1686]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1687]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1688]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1689]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1690]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1691]*acadoWorkspace.Dx0[11] + acadoVariables.x[152];
tmp += acadoWorkspace.d[140];
acadoWorkspace.lbA[35] = - tmp;
acadoWorkspace.ubA[35] = - tmp;
tmp = + acadoWorkspace.evGx[1800]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1801]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1802]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1803]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1804]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1805]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1806]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1807]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1808]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1809]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1810]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1811]*acadoWorkspace.Dx0[11] + acadoVariables.x[162];
tmp += acadoWorkspace.d[150];
acadoWorkspace.lbA[36] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[36] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1812]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1813]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1814]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1815]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1816]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1817]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1818]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1819]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1820]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1821]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1822]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1823]*acadoWorkspace.Dx0[11] + acadoVariables.x[163];
tmp += acadoWorkspace.d[151];
acadoWorkspace.lbA[37] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[37] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1824]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1825]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1826]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1827]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1828]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1829]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1830]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1831]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1832]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1833]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1834]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1835]*acadoWorkspace.Dx0[11] + acadoVariables.x[164];
tmp += acadoWorkspace.d[152];
acadoWorkspace.lbA[38] = - tmp;
acadoWorkspace.ubA[38] = - tmp;
tmp = + acadoWorkspace.evGx[1944]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1945]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1946]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1947]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1948]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1949]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1950]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1951]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1952]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1953]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1954]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1955]*acadoWorkspace.Dx0[11] + acadoVariables.x[174];
tmp += acadoWorkspace.d[162];
acadoWorkspace.lbA[39] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[39] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1956]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1957]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1958]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1959]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1960]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1961]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1962]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1963]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1964]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1965]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1966]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1967]*acadoWorkspace.Dx0[11] + acadoVariables.x[175];
tmp += acadoWorkspace.d[163];
acadoWorkspace.lbA[40] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[40] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[1968]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1969]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[1970]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[1971]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[1972]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[1973]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[1974]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[1975]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[1976]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[1977]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[1978]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[1979]*acadoWorkspace.Dx0[11] + acadoVariables.x[176];
tmp += acadoWorkspace.d[164];
acadoWorkspace.lbA[41] = - tmp;
acadoWorkspace.ubA[41] = - tmp;
tmp = + acadoWorkspace.evGx[2088]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2089]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2090]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2091]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2092]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2093]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2094]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2095]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2096]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2097]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2098]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2099]*acadoWorkspace.Dx0[11] + acadoVariables.x[186];
tmp += acadoWorkspace.d[174];
acadoWorkspace.lbA[42] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[42] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2100]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2101]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2102]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2103]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2104]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2105]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2106]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2107]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2108]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2109]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2110]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2111]*acadoWorkspace.Dx0[11] + acadoVariables.x[187];
tmp += acadoWorkspace.d[175];
acadoWorkspace.lbA[43] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[43] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2112]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2113]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2114]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2115]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2116]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2117]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2118]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2119]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2120]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2121]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2122]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2123]*acadoWorkspace.Dx0[11] + acadoVariables.x[188];
tmp += acadoWorkspace.d[176];
acadoWorkspace.lbA[44] = - tmp;
acadoWorkspace.ubA[44] = - tmp;
tmp = + acadoWorkspace.evGx[2232]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2233]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2234]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2235]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2236]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2237]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2238]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2239]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2240]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2241]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2242]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2243]*acadoWorkspace.Dx0[11] + acadoVariables.x[198];
tmp += acadoWorkspace.d[186];
acadoWorkspace.lbA[45] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[45] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2244]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2245]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2246]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2247]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2248]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2249]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2250]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2251]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2252]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2253]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2254]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2255]*acadoWorkspace.Dx0[11] + acadoVariables.x[199];
tmp += acadoWorkspace.d[187];
acadoWorkspace.lbA[46] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[46] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2256]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2257]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2258]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2259]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2260]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2261]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2262]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2263]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2264]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2265]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2266]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2267]*acadoWorkspace.Dx0[11] + acadoVariables.x[200];
tmp += acadoWorkspace.d[188];
acadoWorkspace.lbA[47] = - tmp;
acadoWorkspace.ubA[47] = - tmp;
tmp = + acadoWorkspace.evGx[2376]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2377]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2378]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2379]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2380]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2381]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2382]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2383]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2384]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2385]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2386]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2387]*acadoWorkspace.Dx0[11] + acadoVariables.x[210];
tmp += acadoWorkspace.d[198];
acadoWorkspace.lbA[48] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[48] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2388]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2389]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2390]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2391]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2392]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2393]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2394]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2395]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2396]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2397]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2398]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2399]*acadoWorkspace.Dx0[11] + acadoVariables.x[211];
tmp += acadoWorkspace.d[199];
acadoWorkspace.lbA[49] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[49] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2400]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2401]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2402]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2403]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2404]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2405]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2406]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2407]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2408]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2409]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2410]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2411]*acadoWorkspace.Dx0[11] + acadoVariables.x[212];
tmp += acadoWorkspace.d[200];
acadoWorkspace.lbA[50] = - tmp;
acadoWorkspace.ubA[50] = - tmp;
tmp = + acadoWorkspace.evGx[2520]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2521]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2522]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2523]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2524]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2525]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2526]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2527]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2528]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2529]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2530]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2531]*acadoWorkspace.Dx0[11] + acadoVariables.x[222];
tmp += acadoWorkspace.d[210];
acadoWorkspace.lbA[51] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[51] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2532]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2533]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2534]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2535]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2536]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2537]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2538]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2539]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2540]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2541]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2542]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2543]*acadoWorkspace.Dx0[11] + acadoVariables.x[223];
tmp += acadoWorkspace.d[211];
acadoWorkspace.lbA[52] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[52] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2544]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2545]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2546]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2547]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2548]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2549]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2550]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2551]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2552]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2553]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2554]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2555]*acadoWorkspace.Dx0[11] + acadoVariables.x[224];
tmp += acadoWorkspace.d[212];
acadoWorkspace.lbA[53] = - tmp;
acadoWorkspace.ubA[53] = - tmp;
tmp = + acadoWorkspace.evGx[2664]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2665]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2666]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2667]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2668]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2669]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2670]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2671]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2672]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2673]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2674]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2675]*acadoWorkspace.Dx0[11] + acadoVariables.x[234];
tmp += acadoWorkspace.d[222];
acadoWorkspace.lbA[54] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[54] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2676]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2677]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2678]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2679]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2680]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2681]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2682]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2683]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2684]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2685]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2686]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2687]*acadoWorkspace.Dx0[11] + acadoVariables.x[235];
tmp += acadoWorkspace.d[223];
acadoWorkspace.lbA[55] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[55] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2688]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2689]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2690]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2691]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2692]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2693]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2694]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2695]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2696]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2697]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2698]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2699]*acadoWorkspace.Dx0[11] + acadoVariables.x[236];
tmp += acadoWorkspace.d[224];
acadoWorkspace.lbA[56] = - tmp;
acadoWorkspace.ubA[56] = - tmp;
tmp = + acadoWorkspace.evGx[2808]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2809]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2810]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2811]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2812]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2813]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2814]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2815]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2816]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2817]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2818]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2819]*acadoWorkspace.Dx0[11] + acadoVariables.x[246];
tmp += acadoWorkspace.d[234];
acadoWorkspace.lbA[57] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[57] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2820]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2821]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2822]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2823]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2824]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2825]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2826]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2827]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2828]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2829]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2830]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2831]*acadoWorkspace.Dx0[11] + acadoVariables.x[247];
tmp += acadoWorkspace.d[235];
acadoWorkspace.lbA[58] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[58] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2832]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2833]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2834]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2835]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2836]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2837]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2838]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2839]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2840]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2841]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2842]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2843]*acadoWorkspace.Dx0[11] + acadoVariables.x[248];
tmp += acadoWorkspace.d[236];
acadoWorkspace.lbA[59] = - tmp;
acadoWorkspace.ubA[59] = - tmp;
tmp = + acadoWorkspace.evGx[2952]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2953]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2954]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2955]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2956]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2957]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2958]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2959]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2960]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2961]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2962]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2963]*acadoWorkspace.Dx0[11] + acadoVariables.x[258];
tmp += acadoWorkspace.d[246];
acadoWorkspace.lbA[60] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[60] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2964]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2965]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2966]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2967]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2968]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2969]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2970]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2971]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2972]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2973]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2974]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2975]*acadoWorkspace.Dx0[11] + acadoVariables.x[259];
tmp += acadoWorkspace.d[247];
acadoWorkspace.lbA[61] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[61] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[2976]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[2977]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2978]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[2979]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[2980]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[2981]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[2982]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[2983]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[2984]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[2985]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[2986]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[2987]*acadoWorkspace.Dx0[11] + acadoVariables.x[260];
tmp += acadoWorkspace.d[248];
acadoWorkspace.lbA[62] = - tmp;
acadoWorkspace.ubA[62] = - tmp;
tmp = + acadoWorkspace.evGx[3096]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3097]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3098]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3099]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3100]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3101]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3102]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3103]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3104]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3105]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3106]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3107]*acadoWorkspace.Dx0[11] + acadoVariables.x[270];
tmp += acadoWorkspace.d[258];
acadoWorkspace.lbA[63] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[63] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3111]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3112]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3113]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3114]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3115]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3116]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3117]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3118]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3119]*acadoWorkspace.Dx0[11] + acadoVariables.x[271];
tmp += acadoWorkspace.d[259];
acadoWorkspace.lbA[64] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[64] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3122]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3123]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3124]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3125]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3126]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3127]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3128]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3129]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3130]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3131]*acadoWorkspace.Dx0[11] + acadoVariables.x[272];
tmp += acadoWorkspace.d[260];
acadoWorkspace.lbA[65] = - tmp;
acadoWorkspace.ubA[65] = - tmp;
tmp = + acadoWorkspace.evGx[3240]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3241]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3242]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3243]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3244]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3245]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3246]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3247]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3248]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3249]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3250]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3251]*acadoWorkspace.Dx0[11] + acadoVariables.x[282];
tmp += acadoWorkspace.d[270];
acadoWorkspace.lbA[66] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[66] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3255]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3256]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3257]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3258]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3259]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3260]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3261]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3262]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3263]*acadoWorkspace.Dx0[11] + acadoVariables.x[283];
tmp += acadoWorkspace.d[271];
acadoWorkspace.lbA[67] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[67] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3264]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3265]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3266]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3267]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3268]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3269]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3270]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3271]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3272]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3273]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3274]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3275]*acadoWorkspace.Dx0[11] + acadoVariables.x[284];
tmp += acadoWorkspace.d[272];
acadoWorkspace.lbA[68] = - tmp;
acadoWorkspace.ubA[68] = - tmp;
tmp = + acadoWorkspace.evGx[3384]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3385]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3386]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3387]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3388]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3389]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3390]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3391]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3392]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3393]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3394]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3395]*acadoWorkspace.Dx0[11] + acadoVariables.x[294];
tmp += acadoWorkspace.d[282];
acadoWorkspace.lbA[69] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[69] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3396]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3397]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3398]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3399]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3400]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3401]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3402]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3403]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3404]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3405]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3406]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3407]*acadoWorkspace.Dx0[11] + acadoVariables.x[295];
tmp += acadoWorkspace.d[283];
acadoWorkspace.lbA[70] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[70] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3408]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3409]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3410]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3411]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3412]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3413]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3414]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3415]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3416]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3417]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3418]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3419]*acadoWorkspace.Dx0[11] + acadoVariables.x[296];
tmp += acadoWorkspace.d[284];
acadoWorkspace.lbA[71] = - tmp;
acadoWorkspace.ubA[71] = - tmp;
tmp = + acadoWorkspace.evGx[3528]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3529]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3530]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3531]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3532]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3533]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3534]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3535]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3536]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3537]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3538]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3539]*acadoWorkspace.Dx0[11] + acadoVariables.x[306];
tmp += acadoWorkspace.d[294];
acadoWorkspace.lbA[72] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[72] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3540]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3541]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3542]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3543]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3544]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3545]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3546]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3547]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3548]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3549]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3550]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3551]*acadoWorkspace.Dx0[11] + acadoVariables.x[307];
tmp += acadoWorkspace.d[295];
acadoWorkspace.lbA[73] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[73] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3552]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3553]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3554]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3555]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3556]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3557]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3558]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3559]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3560]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3561]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3562]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3563]*acadoWorkspace.Dx0[11] + acadoVariables.x[308];
tmp += acadoWorkspace.d[296];
acadoWorkspace.lbA[74] = - tmp;
acadoWorkspace.ubA[74] = - tmp;
tmp = + acadoWorkspace.evGx[3672]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3673]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3674]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3675]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3676]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3677]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3678]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3679]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3680]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3681]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3682]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3683]*acadoWorkspace.Dx0[11] + acadoVariables.x[318];
tmp += acadoWorkspace.d[306];
acadoWorkspace.lbA[75] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[75] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3684]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3685]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3686]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3687]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3688]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3689]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3690]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3691]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3692]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3693]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3694]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3695]*acadoWorkspace.Dx0[11] + acadoVariables.x[319];
tmp += acadoWorkspace.d[307];
acadoWorkspace.lbA[76] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[76] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3696]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3697]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3698]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3699]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3700]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3701]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3702]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3703]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3704]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3705]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3706]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3707]*acadoWorkspace.Dx0[11] + acadoVariables.x[320];
tmp += acadoWorkspace.d[308];
acadoWorkspace.lbA[77] = - tmp;
acadoWorkspace.ubA[77] = - tmp;
tmp = + acadoWorkspace.evGx[3816]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3817]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3818]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3819]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3820]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3821]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3822]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3823]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3824]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3825]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3826]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3827]*acadoWorkspace.Dx0[11] + acadoVariables.x[330];
tmp += acadoWorkspace.d[318];
acadoWorkspace.lbA[78] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[78] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3828]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3829]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3830]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3831]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3832]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3833]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3834]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3835]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3836]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3837]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3838]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3839]*acadoWorkspace.Dx0[11] + acadoVariables.x[331];
tmp += acadoWorkspace.d[319];
acadoWorkspace.lbA[79] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[79] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3840]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3841]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3842]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3843]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3844]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3845]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3846]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3847]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3848]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3849]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3850]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3851]*acadoWorkspace.Dx0[11] + acadoVariables.x[332];
tmp += acadoWorkspace.d[320];
acadoWorkspace.lbA[80] = - tmp;
acadoWorkspace.ubA[80] = - tmp;
tmp = + acadoWorkspace.evGx[3960]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3961]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3962]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3963]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3964]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3965]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3966]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3967]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3968]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3969]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3970]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3971]*acadoWorkspace.Dx0[11] + acadoVariables.x[342];
tmp += acadoWorkspace.d[330];
acadoWorkspace.lbA[81] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[81] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3972]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3973]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3974]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3975]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3976]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3977]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3978]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3979]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3980]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3981]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3982]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3983]*acadoWorkspace.Dx0[11] + acadoVariables.x[343];
tmp += acadoWorkspace.d[331];
acadoWorkspace.lbA[82] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[82] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[3984]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[3985]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[3986]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3987]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[3988]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[3989]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[3990]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[3991]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[3992]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[3993]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[3994]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[3995]*acadoWorkspace.Dx0[11] + acadoVariables.x[344];
tmp += acadoWorkspace.d[332];
acadoWorkspace.lbA[83] = - tmp;
acadoWorkspace.ubA[83] = - tmp;
tmp = + acadoWorkspace.evGx[4104]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4105]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[4106]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[4107]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4108]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[4109]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[4110]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[4111]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[4112]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[4113]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[4114]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[4115]*acadoWorkspace.Dx0[11] + acadoVariables.x[354];
tmp += acadoWorkspace.d[342];
acadoWorkspace.lbA[84] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[84] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[4116]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4117]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[4118]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[4119]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4120]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[4121]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[4122]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[4123]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[4124]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[4125]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[4126]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[4127]*acadoWorkspace.Dx0[11] + acadoVariables.x[355];
tmp += acadoWorkspace.d[343];
acadoWorkspace.lbA[85] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[85] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[4128]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4129]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[4130]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[4131]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4132]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[4133]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[4134]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[4135]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[4136]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[4137]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[4138]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[4139]*acadoWorkspace.Dx0[11] + acadoVariables.x[356];
tmp += acadoWorkspace.d[344];
acadoWorkspace.lbA[86] = - tmp;
acadoWorkspace.ubA[86] = - tmp;
tmp = + acadoWorkspace.evGx[4248]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4249]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[4250]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[4251]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4252]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[4253]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[4254]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[4255]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[4256]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[4257]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[4258]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[4259]*acadoWorkspace.Dx0[11] + acadoVariables.x[366];
tmp += acadoWorkspace.d[354];
acadoWorkspace.lbA[87] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[87] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[4260]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4261]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[4262]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[4263]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4264]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[4265]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[4266]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[4267]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[4268]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[4269]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[4270]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[4271]*acadoWorkspace.Dx0[11] + acadoVariables.x[367];
tmp += acadoWorkspace.d[355];
acadoWorkspace.lbA[88] = (real_t)-2.9872299859827667e-01 - tmp;
acadoWorkspace.ubA[88] = (real_t)2.9872299859827667e-01 - tmp;
tmp = + acadoWorkspace.evGx[4272]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4273]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[4274]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[4275]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4276]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[4277]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[4278]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[4279]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[4280]*acadoWorkspace.Dx0[8] + acadoWorkspace.evGx[4281]*acadoWorkspace.Dx0[9] + acadoWorkspace.evGx[4282]*acadoWorkspace.Dx0[10] + acadoWorkspace.evGx[4283]*acadoWorkspace.Dx0[11] + acadoVariables.x[368];
tmp += acadoWorkspace.d[356];
acadoWorkspace.lbA[89] = - tmp;
acadoWorkspace.ubA[89] = - tmp;

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[200]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[201]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[202]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[203]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[206]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[207]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[208]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[209]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[210]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[211]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[212]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[213]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[214]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[215]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[221]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[222]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[223]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[224]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[225]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[226]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[227]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[230]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[231]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[232]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[233]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[234]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[235]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[236]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[237]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[238]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[239]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[240]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[241]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[242]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[243]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[244]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[245]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[246]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[247]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[248]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[249]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[250]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[251]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[256]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[257]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[258]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[259]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[260]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[261]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[262]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[263]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[264]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[265]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[266]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[267]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[268]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[269]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[270]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[271]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[272]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[273]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[274]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[275]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[276]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[277]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[278]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[279]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[280]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[281]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[282]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[283]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[284]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[285]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[286]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[287]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[292]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[293]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[294]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[295]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[296]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[297]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[298]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[299]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[300]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[301]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[302]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[303]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[304]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[305]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[306]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[307]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[308]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[309]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[310]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[311]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[312]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[313]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[314]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[315]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[316]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[317]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[318]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[319]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[320]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[321]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[322]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[323]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[327]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[328]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[329]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[330]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[331]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[332]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[333]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[334]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[335]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[336]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[337]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[338]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[339]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[340]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[341]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[342]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[343]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[344]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[345]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[346]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[347]*acadoWorkspace.Dx0[11];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[348]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[349]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[350]*acadoWorkspace.Dx0[2] + acadoWorkspace.A01[351]*acadoWorkspace.Dx0[3] + acadoWorkspace.A01[352]*acadoWorkspace.Dx0[4] + acadoWorkspace.A01[353]*acadoWorkspace.Dx0[5] + acadoWorkspace.A01[354]*acadoWorkspace.Dx0[6] + acadoWorkspace.A01[355]*acadoWorkspace.Dx0[7] + acadoWorkspace.A01[356]*acadoWorkspace.Dx0[8] + acadoWorkspace.A01[357]*acadoWorkspace.Dx0[9] + acadoWorkspace.A01[358]*acadoWorkspace.Dx0[10] + acadoWorkspace.A01[359]*acadoWorkspace.Dx0[11];
acadoWorkspace.lbA[90] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[91] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[92] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[93] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[94] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[95] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[96] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[97] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[98] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[99] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[100] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[101] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[102] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[103] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[104] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[105] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[106] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[107] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[108] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[109] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.lbA[110] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.lbA[111] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.lbA[112] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.lbA[113] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.lbA[114] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.lbA[115] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.lbA[116] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.lbA[117] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.lbA[118] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.lbA[119] -= acadoWorkspace.pacA01Dx0[29];

acadoWorkspace.ubA[90] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[91] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[92] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[93] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[94] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[95] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[96] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[97] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[98] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[99] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[100] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[101] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[102] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[103] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[104] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[105] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[106] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[107] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[108] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[109] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.ubA[110] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.ubA[111] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.ubA[112] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.ubA[113] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.ubA[114] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.ubA[115] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.ubA[116] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.ubA[117] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.ubA[118] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.ubA[119] -= acadoWorkspace.pacA01Dx0[29];

}

void acado_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoVariables.u[80] += acadoWorkspace.x[80];
acadoVariables.u[81] += acadoWorkspace.x[81];
acadoVariables.u[82] += acadoWorkspace.x[82];
acadoVariables.u[83] += acadoWorkspace.x[83];
acadoVariables.u[84] += acadoWorkspace.x[84];
acadoVariables.u[85] += acadoWorkspace.x[85];
acadoVariables.u[86] += acadoWorkspace.x[86];
acadoVariables.u[87] += acadoWorkspace.x[87];
acadoVariables.u[88] += acadoWorkspace.x[88];
acadoVariables.u[89] += acadoWorkspace.x[89];
acadoVariables.u[90] += acadoWorkspace.x[90];
acadoVariables.u[91] += acadoWorkspace.x[91];
acadoVariables.u[92] += acadoWorkspace.x[92];
acadoVariables.u[93] += acadoWorkspace.x[93];
acadoVariables.u[94] += acadoWorkspace.x[94];
acadoVariables.u[95] += acadoWorkspace.x[95];
acadoVariables.u[96] += acadoWorkspace.x[96];
acadoVariables.u[97] += acadoWorkspace.x[97];
acadoVariables.u[98] += acadoWorkspace.x[98];
acadoVariables.u[99] += acadoWorkspace.x[99];
acadoVariables.u[100] += acadoWorkspace.x[100];
acadoVariables.u[101] += acadoWorkspace.x[101];
acadoVariables.u[102] += acadoWorkspace.x[102];
acadoVariables.u[103] += acadoWorkspace.x[103];
acadoVariables.u[104] += acadoWorkspace.x[104];
acadoVariables.u[105] += acadoWorkspace.x[105];
acadoVariables.u[106] += acadoWorkspace.x[106];
acadoVariables.u[107] += acadoWorkspace.x[107];
acadoVariables.u[108] += acadoWorkspace.x[108];
acadoVariables.u[109] += acadoWorkspace.x[109];
acadoVariables.u[110] += acadoWorkspace.x[110];
acadoVariables.u[111] += acadoWorkspace.x[111];
acadoVariables.u[112] += acadoWorkspace.x[112];
acadoVariables.u[113] += acadoWorkspace.x[113];
acadoVariables.u[114] += acadoWorkspace.x[114];
acadoVariables.u[115] += acadoWorkspace.x[115];
acadoVariables.u[116] += acadoWorkspace.x[116];
acadoVariables.u[117] += acadoWorkspace.x[117];
acadoVariables.u[118] += acadoWorkspace.x[118];
acadoVariables.u[119] += acadoWorkspace.x[119];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];
acadoVariables.x[5] += acadoWorkspace.Dx0[5];
acadoVariables.x[6] += acadoWorkspace.Dx0[6];
acadoVariables.x[7] += acadoWorkspace.Dx0[7];
acadoVariables.x[8] += acadoWorkspace.Dx0[8];
acadoVariables.x[9] += acadoWorkspace.Dx0[9];
acadoVariables.x[10] += acadoWorkspace.Dx0[10];
acadoVariables.x[11] += acadoWorkspace.Dx0[11];

for (lRun1 = 0; lRun1 < 360; ++lRun1)
{
for (lRun2 = 0; lRun2 < 1; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 12; ++lRun3)
{
t += + acadoWorkspace.evGx[(lRun1 * 12) + (lRun3)]*acadoWorkspace.Dx0[(lRun3) + (lRun2)];
}
acadoVariables.x[(lRun1 + 12) + (lRun2)] += t + acadoWorkspace.d[(lRun1) + (lRun2)];
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
acado_multEDu( &(acadoWorkspace.E[ lRun3 * 48 ]), &(acadoWorkspace.x[ lRun2 * 4 ]), &(acadoVariables.x[ lRun1 * 12 + 12 ]) );
}
}
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 12];
acadoWorkspace.state[1] = acadoVariables.x[index * 12 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 12 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 12 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 12 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 12 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 12 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 12 + 7];
acadoWorkspace.state[8] = acadoVariables.x[index * 12 + 8];
acadoWorkspace.state[9] = acadoVariables.x[index * 12 + 9];
acadoWorkspace.state[10] = acadoVariables.x[index * 12 + 10];
acadoWorkspace.state[11] = acadoVariables.x[index * 12 + 11];
acadoWorkspace.state[204] = acadoVariables.u[index * 4];
acadoWorkspace.state[205] = acadoVariables.u[index * 4 + 1];
acadoWorkspace.state[206] = acadoVariables.u[index * 4 + 2];
acadoWorkspace.state[207] = acadoVariables.u[index * 4 + 3];
acadoWorkspace.state[208] = acadoVariables.od[index];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 12 + 12] = acadoWorkspace.state[0];
acadoVariables.x[index * 12 + 13] = acadoWorkspace.state[1];
acadoVariables.x[index * 12 + 14] = acadoWorkspace.state[2];
acadoVariables.x[index * 12 + 15] = acadoWorkspace.state[3];
acadoVariables.x[index * 12 + 16] = acadoWorkspace.state[4];
acadoVariables.x[index * 12 + 17] = acadoWorkspace.state[5];
acadoVariables.x[index * 12 + 18] = acadoWorkspace.state[6];
acadoVariables.x[index * 12 + 19] = acadoWorkspace.state[7];
acadoVariables.x[index * 12 + 20] = acadoWorkspace.state[8];
acadoVariables.x[index * 12 + 21] = acadoWorkspace.state[9];
acadoVariables.x[index * 12 + 22] = acadoWorkspace.state[10];
acadoVariables.x[index * 12 + 23] = acadoWorkspace.state[11];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoVariables.x[index * 12] = acadoVariables.x[index * 12 + 12];
acadoVariables.x[index * 12 + 1] = acadoVariables.x[index * 12 + 13];
acadoVariables.x[index * 12 + 2] = acadoVariables.x[index * 12 + 14];
acadoVariables.x[index * 12 + 3] = acadoVariables.x[index * 12 + 15];
acadoVariables.x[index * 12 + 4] = acadoVariables.x[index * 12 + 16];
acadoVariables.x[index * 12 + 5] = acadoVariables.x[index * 12 + 17];
acadoVariables.x[index * 12 + 6] = acadoVariables.x[index * 12 + 18];
acadoVariables.x[index * 12 + 7] = acadoVariables.x[index * 12 + 19];
acadoVariables.x[index * 12 + 8] = acadoVariables.x[index * 12 + 20];
acadoVariables.x[index * 12 + 9] = acadoVariables.x[index * 12 + 21];
acadoVariables.x[index * 12 + 10] = acadoVariables.x[index * 12 + 22];
acadoVariables.x[index * 12 + 11] = acadoVariables.x[index * 12 + 23];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[360] = xEnd[0];
acadoVariables.x[361] = xEnd[1];
acadoVariables.x[362] = xEnd[2];
acadoVariables.x[363] = xEnd[3];
acadoVariables.x[364] = xEnd[4];
acadoVariables.x[365] = xEnd[5];
acadoVariables.x[366] = xEnd[6];
acadoVariables.x[367] = xEnd[7];
acadoVariables.x[368] = xEnd[8];
acadoVariables.x[369] = xEnd[9];
acadoVariables.x[370] = xEnd[10];
acadoVariables.x[371] = xEnd[11];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[360];
acadoWorkspace.state[1] = acadoVariables.x[361];
acadoWorkspace.state[2] = acadoVariables.x[362];
acadoWorkspace.state[3] = acadoVariables.x[363];
acadoWorkspace.state[4] = acadoVariables.x[364];
acadoWorkspace.state[5] = acadoVariables.x[365];
acadoWorkspace.state[6] = acadoVariables.x[366];
acadoWorkspace.state[7] = acadoVariables.x[367];
acadoWorkspace.state[8] = acadoVariables.x[368];
acadoWorkspace.state[9] = acadoVariables.x[369];
acadoWorkspace.state[10] = acadoVariables.x[370];
acadoWorkspace.state[11] = acadoVariables.x[371];
if (uEnd != 0)
{
acadoWorkspace.state[204] = uEnd[0];
acadoWorkspace.state[205] = uEnd[1];
acadoWorkspace.state[206] = uEnd[2];
acadoWorkspace.state[207] = uEnd[3];
}
else
{
acadoWorkspace.state[204] = acadoVariables.u[116];
acadoWorkspace.state[205] = acadoVariables.u[117];
acadoWorkspace.state[206] = acadoVariables.u[118];
acadoWorkspace.state[207] = acadoVariables.u[119];
}
acadoWorkspace.state[208] = acadoVariables.od[30];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[360] = acadoWorkspace.state[0];
acadoVariables.x[361] = acadoWorkspace.state[1];
acadoVariables.x[362] = acadoWorkspace.state[2];
acadoVariables.x[363] = acadoWorkspace.state[3];
acadoVariables.x[364] = acadoWorkspace.state[4];
acadoVariables.x[365] = acadoWorkspace.state[5];
acadoVariables.x[366] = acadoWorkspace.state[6];
acadoVariables.x[367] = acadoWorkspace.state[7];
acadoVariables.x[368] = acadoWorkspace.state[8];
acadoVariables.x[369] = acadoWorkspace.state[9];
acadoVariables.x[370] = acadoWorkspace.state[10];
acadoVariables.x[371] = acadoWorkspace.state[11];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[116] = uEnd[0];
acadoVariables.u[117] = uEnd[1];
acadoVariables.u[118] = uEnd[2];
acadoVariables.u[119] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110] + acadoWorkspace.g[111]*acadoWorkspace.x[111] + acadoWorkspace.g[112]*acadoWorkspace.x[112] + acadoWorkspace.g[113]*acadoWorkspace.x[113] + acadoWorkspace.g[114]*acadoWorkspace.x[114] + acadoWorkspace.g[115]*acadoWorkspace.x[115] + acadoWorkspace.g[116]*acadoWorkspace.x[116] + acadoWorkspace.g[117]*acadoWorkspace.x[117] + acadoWorkspace.g[118]*acadoWorkspace.x[118] + acadoWorkspace.g[119]*acadoWorkspace.x[119];
kkt = fabs( kkt );
for (index = 0; index < 120; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 120; ++index)
{
prd = acadoWorkspace.y[index + 120];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 10 */
real_t tmpDy[ 10 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 12];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 12 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 12 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 12 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 12 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 12 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 12 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 12 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 12 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 12 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[lRun1 * 12 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[lRun1 * 12 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[13] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 10] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 10];
acadoWorkspace.Dy[lRun1 * 10 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 10 + 1];
acadoWorkspace.Dy[lRun1 * 10 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 10 + 2];
acadoWorkspace.Dy[lRun1 * 10 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 10 + 3];
acadoWorkspace.Dy[lRun1 * 10 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 10 + 4];
acadoWorkspace.Dy[lRun1 * 10 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 10 + 5];
acadoWorkspace.Dy[lRun1 * 10 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 10 + 6];
acadoWorkspace.Dy[lRun1 * 10 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 10 + 7];
acadoWorkspace.Dy[lRun1 * 10 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 10 + 8];
acadoWorkspace.Dy[lRun1 * 10 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 10 + 9];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[360];
acadoWorkspace.objValueIn[1] = acadoVariables.x[361];
acadoWorkspace.objValueIn[2] = acadoVariables.x[362];
acadoWorkspace.objValueIn[3] = acadoVariables.x[363];
acadoWorkspace.objValueIn[4] = acadoVariables.x[364];
acadoWorkspace.objValueIn[5] = acadoVariables.x[365];
acadoWorkspace.objValueIn[6] = acadoVariables.x[366];
acadoWorkspace.objValueIn[7] = acadoVariables.x[367];
acadoWorkspace.objValueIn[8] = acadoVariables.x[368];
acadoWorkspace.objValueIn[9] = acadoVariables.x[369];
acadoWorkspace.objValueIn[10] = acadoVariables.x[370];
acadoWorkspace.objValueIn[11] = acadoVariables.x[371];
acadoWorkspace.objValueIn[12] = acadoVariables.od[30];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 10]*acadoVariables.W[lRun1 * 100] + acadoWorkspace.Dy[lRun1 * 10 + 1]*acadoVariables.W[lRun1 * 100 + 10] + acadoWorkspace.Dy[lRun1 * 10 + 2]*acadoVariables.W[lRun1 * 100 + 20] + acadoWorkspace.Dy[lRun1 * 10 + 3]*acadoVariables.W[lRun1 * 100 + 30] + acadoWorkspace.Dy[lRun1 * 10 + 4]*acadoVariables.W[lRun1 * 100 + 40] + acadoWorkspace.Dy[lRun1 * 10 + 5]*acadoVariables.W[lRun1 * 100 + 50] + acadoWorkspace.Dy[lRun1 * 10 + 6]*acadoVariables.W[lRun1 * 100 + 60] + acadoWorkspace.Dy[lRun1 * 10 + 7]*acadoVariables.W[lRun1 * 100 + 70] + acadoWorkspace.Dy[lRun1 * 10 + 8]*acadoVariables.W[lRun1 * 100 + 80] + acadoWorkspace.Dy[lRun1 * 10 + 9]*acadoVariables.W[lRun1 * 100 + 90];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 10]*acadoVariables.W[lRun1 * 100 + 1] + acadoWorkspace.Dy[lRun1 * 10 + 1]*acadoVariables.W[lRun1 * 100 + 11] + acadoWorkspace.Dy[lRun1 * 10 + 2]*acadoVariables.W[lRun1 * 100 + 21] + acadoWorkspace.Dy[lRun1 * 10 + 3]*acadoVariables.W[lRun1 * 100 + 31] + acadoWorkspace.Dy[lRun1 * 10 + 4]*acadoVariables.W[lRun1 * 100 + 41] + acadoWorkspace.Dy[lRun1 * 10 + 5]*acadoVariables.W[lRun1 * 100 + 51] + acadoWorkspace.Dy[lRun1 * 10 + 6]*acadoVariables.W[lRun1 * 100 + 61] + acadoWorkspace.Dy[lRun1 * 10 + 7]*acadoVariables.W[lRun1 * 100 + 71] + acadoWorkspace.Dy[lRun1 * 10 + 8]*acadoVariables.W[lRun1 * 100 + 81] + acadoWorkspace.Dy[lRun1 * 10 + 9]*acadoVariables.W[lRun1 * 100 + 91];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 10]*acadoVariables.W[lRun1 * 100 + 2] + acadoWorkspace.Dy[lRun1 * 10 + 1]*acadoVariables.W[lRun1 * 100 + 12] + acadoWorkspace.Dy[lRun1 * 10 + 2]*acadoVariables.W[lRun1 * 100 + 22] + acadoWorkspace.Dy[lRun1 * 10 + 3]*acadoVariables.W[lRun1 * 100 + 32] + acadoWorkspace.Dy[lRun1 * 10 + 4]*acadoVariables.W[lRun1 * 100 + 42] + acadoWorkspace.Dy[lRun1 * 10 + 5]*acadoVariables.W[lRun1 * 100 + 52] + acadoWorkspace.Dy[lRun1 * 10 + 6]*acadoVariables.W[lRun1 * 100 + 62] + acadoWorkspace.Dy[lRun1 * 10 + 7]*acadoVariables.W[lRun1 * 100 + 72] + acadoWorkspace.Dy[lRun1 * 10 + 8]*acadoVariables.W[lRun1 * 100 + 82] + acadoWorkspace.Dy[lRun1 * 10 + 9]*acadoVariables.W[lRun1 * 100 + 92];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 10]*acadoVariables.W[lRun1 * 100 + 3] + acadoWorkspace.Dy[lRun1 * 10 + 1]*acadoVariables.W[lRun1 * 100 + 13] + acadoWorkspace.Dy[lRun1 * 10 + 2]*acadoVariables.W[lRun1 * 100 + 23] + acadoWorkspace.Dy[lRun1 * 10 + 3]*acadoVariables.W[lRun1 * 100 + 33] + acadoWorkspace.Dy[lRun1 * 10 + 4]*acadoVariables.W[lRun1 * 100 + 43] + acadoWorkspace.Dy[lRun1 * 10 + 5]*acadoVariables.W[lRun1 * 100 + 53] + acadoWorkspace.Dy[lRun1 * 10 + 6]*acadoVariables.W[lRun1 * 100 + 63] + acadoWorkspace.Dy[lRun1 * 10 + 7]*acadoVariables.W[lRun1 * 100 + 73] + acadoWorkspace.Dy[lRun1 * 10 + 8]*acadoVariables.W[lRun1 * 100 + 83] + acadoWorkspace.Dy[lRun1 * 10 + 9]*acadoVariables.W[lRun1 * 100 + 93];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 10]*acadoVariables.W[lRun1 * 100 + 4] + acadoWorkspace.Dy[lRun1 * 10 + 1]*acadoVariables.W[lRun1 * 100 + 14] + acadoWorkspace.Dy[lRun1 * 10 + 2]*acadoVariables.W[lRun1 * 100 + 24] + acadoWorkspace.Dy[lRun1 * 10 + 3]*acadoVariables.W[lRun1 * 100 + 34] + acadoWorkspace.Dy[lRun1 * 10 + 4]*acadoVariables.W[lRun1 * 100 + 44] + acadoWorkspace.Dy[lRun1 * 10 + 5]*acadoVariables.W[lRun1 * 100 + 54] + acadoWorkspace.Dy[lRun1 * 10 + 6]*acadoVariables.W[lRun1 * 100 + 64] + acadoWorkspace.Dy[lRun1 * 10 + 7]*acadoVariables.W[lRun1 * 100 + 74] + acadoWorkspace.Dy[lRun1 * 10 + 8]*acadoVariables.W[lRun1 * 100 + 84] + acadoWorkspace.Dy[lRun1 * 10 + 9]*acadoVariables.W[lRun1 * 100 + 94];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 10]*acadoVariables.W[lRun1 * 100 + 5] + acadoWorkspace.Dy[lRun1 * 10 + 1]*acadoVariables.W[lRun1 * 100 + 15] + acadoWorkspace.Dy[lRun1 * 10 + 2]*acadoVariables.W[lRun1 * 100 + 25] + acadoWorkspace.Dy[lRun1 * 10 + 3]*acadoVariables.W[lRun1 * 100 + 35] + acadoWorkspace.Dy[lRun1 * 10 + 4]*acadoVariables.W[lRun1 * 100 + 45] + acadoWorkspace.Dy[lRun1 * 10 + 5]*acadoVariables.W[lRun1 * 100 + 55] + acadoWorkspace.Dy[lRun1 * 10 + 6]*acadoVariables.W[lRun1 * 100 + 65] + acadoWorkspace.Dy[lRun1 * 10 + 7]*acadoVariables.W[lRun1 * 100 + 75] + acadoWorkspace.Dy[lRun1 * 10 + 8]*acadoVariables.W[lRun1 * 100 + 85] + acadoWorkspace.Dy[lRun1 * 10 + 9]*acadoVariables.W[lRun1 * 100 + 95];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 10]*acadoVariables.W[lRun1 * 100 + 6] + acadoWorkspace.Dy[lRun1 * 10 + 1]*acadoVariables.W[lRun1 * 100 + 16] + acadoWorkspace.Dy[lRun1 * 10 + 2]*acadoVariables.W[lRun1 * 100 + 26] + acadoWorkspace.Dy[lRun1 * 10 + 3]*acadoVariables.W[lRun1 * 100 + 36] + acadoWorkspace.Dy[lRun1 * 10 + 4]*acadoVariables.W[lRun1 * 100 + 46] + acadoWorkspace.Dy[lRun1 * 10 + 5]*acadoVariables.W[lRun1 * 100 + 56] + acadoWorkspace.Dy[lRun1 * 10 + 6]*acadoVariables.W[lRun1 * 100 + 66] + acadoWorkspace.Dy[lRun1 * 10 + 7]*acadoVariables.W[lRun1 * 100 + 76] + acadoWorkspace.Dy[lRun1 * 10 + 8]*acadoVariables.W[lRun1 * 100 + 86] + acadoWorkspace.Dy[lRun1 * 10 + 9]*acadoVariables.W[lRun1 * 100 + 96];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 10]*acadoVariables.W[lRun1 * 100 + 7] + acadoWorkspace.Dy[lRun1 * 10 + 1]*acadoVariables.W[lRun1 * 100 + 17] + acadoWorkspace.Dy[lRun1 * 10 + 2]*acadoVariables.W[lRun1 * 100 + 27] + acadoWorkspace.Dy[lRun1 * 10 + 3]*acadoVariables.W[lRun1 * 100 + 37] + acadoWorkspace.Dy[lRun1 * 10 + 4]*acadoVariables.W[lRun1 * 100 + 47] + acadoWorkspace.Dy[lRun1 * 10 + 5]*acadoVariables.W[lRun1 * 100 + 57] + acadoWorkspace.Dy[lRun1 * 10 + 6]*acadoVariables.W[lRun1 * 100 + 67] + acadoWorkspace.Dy[lRun1 * 10 + 7]*acadoVariables.W[lRun1 * 100 + 77] + acadoWorkspace.Dy[lRun1 * 10 + 8]*acadoVariables.W[lRun1 * 100 + 87] + acadoWorkspace.Dy[lRun1 * 10 + 9]*acadoVariables.W[lRun1 * 100 + 97];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 10]*acadoVariables.W[lRun1 * 100 + 8] + acadoWorkspace.Dy[lRun1 * 10 + 1]*acadoVariables.W[lRun1 * 100 + 18] + acadoWorkspace.Dy[lRun1 * 10 + 2]*acadoVariables.W[lRun1 * 100 + 28] + acadoWorkspace.Dy[lRun1 * 10 + 3]*acadoVariables.W[lRun1 * 100 + 38] + acadoWorkspace.Dy[lRun1 * 10 + 4]*acadoVariables.W[lRun1 * 100 + 48] + acadoWorkspace.Dy[lRun1 * 10 + 5]*acadoVariables.W[lRun1 * 100 + 58] + acadoWorkspace.Dy[lRun1 * 10 + 6]*acadoVariables.W[lRun1 * 100 + 68] + acadoWorkspace.Dy[lRun1 * 10 + 7]*acadoVariables.W[lRun1 * 100 + 78] + acadoWorkspace.Dy[lRun1 * 10 + 8]*acadoVariables.W[lRun1 * 100 + 88] + acadoWorkspace.Dy[lRun1 * 10 + 9]*acadoVariables.W[lRun1 * 100 + 98];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 10]*acadoVariables.W[lRun1 * 100 + 9] + acadoWorkspace.Dy[lRun1 * 10 + 1]*acadoVariables.W[lRun1 * 100 + 19] + acadoWorkspace.Dy[lRun1 * 10 + 2]*acadoVariables.W[lRun1 * 100 + 29] + acadoWorkspace.Dy[lRun1 * 10 + 3]*acadoVariables.W[lRun1 * 100 + 39] + acadoWorkspace.Dy[lRun1 * 10 + 4]*acadoVariables.W[lRun1 * 100 + 49] + acadoWorkspace.Dy[lRun1 * 10 + 5]*acadoVariables.W[lRun1 * 100 + 59] + acadoWorkspace.Dy[lRun1 * 10 + 6]*acadoVariables.W[lRun1 * 100 + 69] + acadoWorkspace.Dy[lRun1 * 10 + 7]*acadoVariables.W[lRun1 * 100 + 79] + acadoWorkspace.Dy[lRun1 * 10 + 8]*acadoVariables.W[lRun1 * 100 + 89] + acadoWorkspace.Dy[lRun1 * 10 + 9]*acadoVariables.W[lRun1 * 100 + 99];
objVal += + acadoWorkspace.Dy[lRun1 * 10]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 10 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 10 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 10 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 10 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 10 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 10 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 10 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 10 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 10 + 9]*tmpDy[9];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[7];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[14];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[21];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[28];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[35];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5];

objVal *= 0.5;
return objVal;
}

