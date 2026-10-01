/*
 * AAC HE-AAC v1 (SBR) Encoder
 *
 * Copyright (c) 2026 Nils Schimmelmann
 * Ported to FFmpeg 2026
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/* 31-tap halfband FIR anti-aliasing filter for 2:1 decimation */
static const float fir_halfband[16] = {
    0.50000000f,
    0.31557008f,  0.00000000f, -0.10006240f,  0.00000000f,  0.05260193f,
    0.00000000f, -0.03022513f,  0.00000000f,  0.01712431f,  0.00000000f,
   -0.00888206f,  0.00000000f,  0.00398687f,  0.00000000f, -0.00130986f
};


#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "libavutil/mem.h"
#include "libavutil/mathematics.h"
#include "aacsbr_enc.h"
#include "aacsbrdata.h"
#include "put_bits.h"

typedef struct SBRHuffEntry {
    uint32_t code : 24;
    uint32_t len  : 8;
} SBRHuffEntry;

#define F_HUFF_ENV_1_5DB_OFFSET  60
#define F_HUFF_ENV_1_5DB_NSYMS   121
#define F_HUFF_ENV_3_0DB_OFFSET  31
#define F_HUFF_ENV_3_0DB_NSYMS   63

static const float qmf_c[640] = {
    0.0000000000f, 0.0000000000f, -0.0000000105f, -0.0000000163f, -0.0000000233f, -0.0000000318f, -0.0000000419f, -0.0000000539f,
    -0.0000000678f, -0.0000000839f, -0.0000001024f, -0.0000001233f, -0.0000001469f, -0.0000001733f, -0.0000002027f, -0.0000002353f,
    -0.0000002713f, -0.0000003108f, -0.0000003540f, -0.0000004011f, -0.0000004523f, -0.0000005077f, -0.0000005676f, -0.0000006320f,
    -0.0000007012f, -0.0000007753f, -0.0000008545f, -0.0000009389f, -0.00000010288f, -0.0000011242f, -0.0000012253f, -0.0000013323f,
    -0.0000014453f, -0.0000015645f, -0.0000016899f, -0.0000018218f, -0.0000019602f, -0.0000021053f, -0.0000022572f, -0.0000024160f,
    -0.0000025818f, -0.0000027548f, -0.0000029351f, -0.0000031228f, -0.0000033180f, -0.0000035208f, -0.0000037313f, -0.0000039497f,
    -0.0000041760f, -0.0000044104f, -0.0000046529f, -0.0000049036f, -0.0000051626f, -0.0000054301f, -0.0000057061f, -0.0000059907f,
    -0.0000062840f, -0.0000065861f, -0.0000068971f, -0.0000072170f, -0.0000075460f, -0.0000078841f, -0.0000082313f, -0.0000085878f,
    -0.0000089537f, -0.0000093289f, -0.0000097137f, -0.0000101080f, -0.0000105120f, -0.0000109257f, -0.0000113491f, -0.0000117823f,
    -0.0000122255f, -0.0000126786f, -0.0000131417f, -0.0000136150f, -0.0000140984f, -0.0000145920f, -0.0000150958f, -0.0000156100f,
    -0.0000161345f, -0.0000166695f, -0.0000172150f, -0.0000177710f, -0.0000183377f, -0.0000189150f, -0.0000195030f, -0.0000201019f,
    -0.0000207115f, -0.0000213320f, -0.0000219634f, -0.0000226058f, -0.0000232591f, -0.0000239236f, -0.0000245991f, -0.0000252858f,
    -0.0000259837f, -0.0000266928f, -0.0000274133f, -0.0000281451f, -0.0000288882f, -0.0000296428f, -0.0000304088f, -0.0000311863f,
    -0.0000319754f, -0.0000327760f, -0.0000335883f, -0.0000344122f, -0.0000352478f, -0.0000360951f, -0.0000369542f, -0.0000378250f,
    -0.0000387077f, -0.0000396023f, -0.0000405088f, -0.0000414272f, -0.0000423577f, -0.0000433001f, -0.0000442546f, -0.0000452212f,
    -0.0000461999f, -0.0000471908f, -0.0000481938f, -0.0000492091f, -0.0000502367f, -0.0000512766f, -0.0000523288f, -0.0000533934f,
    0.0000544704f, 0.0000555598f, 0.0000566617f, 0.0000577761f, 0.0000589030f, 0.0000600424f, 0.0000611944f, 0.0000623590f,
    0.0000635362f, 0.0000647260f, 0.0000659285f, 0.0000671437f, 0.0000683716f, 0.0000696123f, 0.0000708657f, 0.0000721319f,
    0.0000734109f, 0.0000747027f, 0.0000760074f, 0.0000773249f, 0.0000786553f, 0.0000799986f, 0.0000813548f, 0.0000827239f,
    0.0000841060f, 0.0000855011f, 0.0000869091f, 0.0000883301f, 0.0000897642f, 0.0000912112f, 0.0000926713f, 0.0000941445f,
    0.0000956307f, 0.0000971301f, 0.0000986425f, 0.0001001681f, 0.0001017068f, 0.0001032587f, 0.0001048238f, 0.0001064020f,
    0.0001079935f, 0.0001095982f, 0.0001112161f, 0.0001128472f, 0.0001144917f, 0.0001161494f, 0.0001178204f, 0.0001195047f,
    0.0001212023f, 0.0001229133f, 0.0001246376f, 0.0001263752f, 0.0001281262f, 0.0001298906f, 0.0001316683f, 0.0001334594f,
    0.0001352639f, 0.0001370817f, 0.0001389130f, 0.0001407576f, 0.0001426156f, 0.0001444870f, 0.0001463718f, 0.0001482700f,
    0.0001501817f, 0.0001521067f, 0.0001540452f, 0.0001559971f, 0.0001579624f, 0.0001599411f, 0.0001619333f, 0.0001639389f,
    0.0001659580f, 0.0001679905f, 0.0001700364f, 0.0001720958f, 0.0001741686f, 0.0001762549f, 0.0001783546f, 0.0001804678f,
    0.0001825944f, 0.0001847345f, 0.0001868880f, 0.0001890550f, 0.0001912354f, 0.0001934293f, 0.0001956367f, 0.0001978575f,
    0.0002000918f, 0.0002023395f, 0.0002046007f, 0.0002068753f, 0.0002091634f, 0.0002114649f, 0.0002137799f, 0.0002161083f,
    0.0002184502f, 0.0002208055f, 0.0002231742f, 0.0002255564f, 0.0002279520f, 0.0002303611f, 0.0002327836f, 0.0002352195f,
    0.0002376689f, 0.0002401317f, 0.0002426079f, 0.0002450976f, 0.0002476007f, 0.0002501172f, 0.0002526471f, 0.0002551905f,
    0.0002577472f, 0.0002603174f, 0.0002629010f, 0.0002654980f, 0.0002681084f, 0.0002707322f, 0.0002733694f, 0.0002760200f,
    0.0002786840f, 0.0002813614f, 0.0002840522f, 0.0002867564f, 0.0002894740f, 0.0002922050f, 0.0002949494f, 0.0002977072f,
    -0.0003004783f, -0.0003032628f, -0.0003060607f, -0.0003088720f, -0.0003116966f, -0.0003145346f, -0.0003173860f, -0.0003202507f,
    -0.0003231288f, -0.0003260203f, -0.0003289251f, -0.0003318433f, -0.0003347748f, -0.0003377197f, -0.0003406779f, -0.0003436495f,
    -0.0003466344f, -0.0003496327f, -0.0003526443f, -0.0003556692f, -0.0003587075f, -0.0003617591f, -0.0003648240f, -0.0003679022f,
    -0.0003709938f, -0.0003740987f, -0.0003772169f, -0.0003803484f, -0.0003834932f, -0.0003866513f, -0.0003898227f, -0.0003930074f,
    -0.0003962054f, -0.0003994167f, -0.0004026413f, -0.0004058792f, -0.0004091304f, -0.0004123949f, -0.0004156727f, -0.0004189638f,
    -0.0004222681f, -0.0004255858f, -0.0004289167f, -0.0004322609f, -0.0004356184f, -0.0004389892f, -0.0004423733f, -0.0004457707f,
    -0.0004491813f, -0.0004526052f, -0.0004560424f, -0.0004594928f, -0.0004629565f, -0.0004664335f, -0.0004699237f, -0.0004734272f,
    -0.0004769440f, -0.0004804740f, -0.0004840173f, -0.0004875738f, -0.0004911436f, -0.0004947267f, -0.0004983230f, -0.0005019325f,
    -0.0005055553f, -0.0005091913f, -0.0005128406f, -0.0005165031f, -0.0005201788f, -0.0005238678f, -0.0005275700f, -0.0005312855f,
    -0.0005350141f, -0.0005387560f, -0.0005425111f, -0.0005462795f, -0.0005500610f, -0.0005538558f, -0.0005576638f, -0.0005614850f,
    -0.0005653195f, -0.0005691672f, -0.0005730281f, -0.0005769022f, -0.0005807895f, -0.0005846900f, -0.0005886038f, -0.0005925307f,
    -0.0005964708f, -0.0006004242f, -0.0006043908f, -0.0006083706f, -0.0006123636f, -0.0006163698f, -0.0006203892f, -0.0006244218f,
    -0.0006284676f, -0.0006325266f, -0.0006365988f, -0.0006406842f, -0.0006447828f, -0.0006488946f, -0.0006530196f, -0.0006571578f,
    -0.0006613092f, -0.0006654737f, -0.0006696515f, -0.0006738424f, -0.0006780465f, -0.0006822638f, -0.0006864943f, -0.0006907380f,
    -0.0006949948f, -0.0006992648f, -0.0007035480f, -0.0007078444f, -0.0007121540f, -0.0007164767f, -0.0007208126f, -0.0007251617f,
    -0.0007295240f, -0.0007338994f, -0.0007382880f, -0.0007426898f, -0.0007471047f, -0.0007515328f, -0.0007559740f, -0.0007604284f,
    0.0007648960f, 0.0007693768f, 0.0007738707f, 0.0007783778f, 0.0007828981f, 0.0007874315f, 0.0007919781f, 0.0007965378f,
    0.0008011107f, 0.0008056968f, 0.0008102960f, 0.0008149084f, 0.0008195339f, 0.0008241726f, 0.0008288244f, 0.0008334894f,
    0.0008381675f, 0.0008428588f, 0.0008475632f, 0.0008522808f, 0.0008570115f, 0.0008617553f, 0.0008665123f, 0.0008712824f,
    0.0008760657f, 0.0008808621f, 0.0008856716f, 0.0008904943f, 0.0008953301f, 0.0009001790f, 0.0009050411f, 0.0009099163f,
    0.0009148046f, 0.0009197061f, 0.0009246207f, 0.0009295484f, 0.0009344892f, 0.0009394431f, 0.0009444102f, 0.0009493904f,
    0.0009543837f, 0.0009593901f, 0.0009644096f, 0.0009694423f, 0.0009744880f, 0.0009795469f, 0.0009846189f, 0.0009897040f,
    0.0009948022f, 0.0009999135f, 0.0010050379f, 0.0010101754f, 0.0010153260f, 0.0010204897f, 0.0010256665f, 0.0010308564f,
    0.0010360594f, 0.0010412755f, 0.0010465047f, 0.0010517470f, 0.0010570024f, 0.0010622709f, 0.0010675525f, 0.0010728472f,
    0.0010781550f, 0.0010834759f, 0.0010888099f, 0.0010941570f, 0.0010995171f, 0.0011048903f, 0.0011102766f, 0.0011156760f,
    0.0011210885f, 0.0011265141f, 0.0011319528f, 0.0011374045f, 0.0011428693f, 0.0011483472f, 0.0011538381f, 0.0011593421f,
    0.0011648592f, 0.0011703894f, 0.0011759326f, 0.0011814889f, 0.0011870583f, 0.0011926407f, 0.0011982362f, 0.0012038448f,
    0.0012094665f, 0.0012151012f, 0.0012207490f, 0.0012264098f, 0.0012320837f, 0.0012377707f, 0.0012434707f, 0.0012491838f,
    0.0012549099f, 0.0012606491f, 0.0012664014f, 0.0012721667f, 0.0012779450f, 0.0012837364f, 0.0012895408f, 0.0012953583f,
    0.0013011888f, 0.0013070324f, 0.0013128890f, 0.0013187586f, 0.0013246413f, 0.0013305370f, 0.0013364457f, 0.0013423675f,
    0.0013483023f, 0.0013542502f, 0.0013602111f, 0.0013661850f, 0.0013721720f, 0.0013781720f, 0.0013841850f, 0.0013902111f,
    0.0013962502f, 0.0014023023f, 0.0014083675f, 0.0014144457f, 0.0014205370f, 0.0014266413f, 0.0014327586f, 0.0014388890f,
    -0.0014450324f, -0.0014511888f, -0.0014573583f, -0.0014635408f, -0.0014697364f, -0.0014759450f, -0.0014821667f, -0.0014884014f,
    -0.0014946491f, -0.0015009099f, -0.0015071838f, -0.0015134707f, -0.0015197707f, -0.0015260837f, -0.0015324098f, -0.0015387490f,
    -0.0015451012f, -0.0015514665f, -0.0015578448f, -0.0015642362f, -0.0015706407f, -0.0015770583f, -0.0015834889f, -0.0015899326f,
    -0.0015963894f, -0.0016028592f, -0.0016093421f, -0.0016158381f, -0.0016223472f, -0.0016288693f, -0.0016354045f, -0.0016419528f,
    -0.0016475141f, -0.0016530885f, -0.0016586760f, -0.0016642766f, -0.0016698903f, -0.0016755171f, -0.0016811570f, -0.0016868099f,
    -0.0016924759f, -0.0016981550f, -0.0017038472f, -0.0017095525f, -0.0017152709f, -0.0017210024f, -0.0017267470f, -0.0017325047f,
    -0.0017382755f, -0.0017440594f, -0.0017498564f, -0.0017556665f, -0.0017614897f, -0.0017673260f, -0.0017731754f, -0.0017790379f,
    -0.0017849135f, -0.0017908022f, -0.0017967040f, -0.0018026189f, -0.0018085469f, -0.0018144880f, -0.0018204423f, -0.0018264096f,
    -0.0018323901f, -0.0018373837f, -0.0018423904f, -0.0018474102f, -0.0018524431f, -0.0018574892f, -0.0018625484f, -0.0018676207f,
    -0.0018727061f, -0.0018778046f, -0.0018829163f, -0.0018880411f, -0.0018931790f, -0.0018983301f, -0.0019034943f, -0.0019086716f,
    -0.0019138621f, -0.0019190657f, -0.0019238324f, -0.0019286123f, -0.0019334053f, -0.0019382115f, -0.0019430308f, -0.0019478632f,
    -0.0019527088f, -0.0019575675f, -0.0019624394f, -0.0019673244f, -0.0019722226f, -0.0019771339f, -0.0019820583f, -0.0019869959f,
    -0.0019919467f, -0.0019969107f, -0.0020018878f, -0.0020068781f, -0.0020118815f, -0.0020168981f, -0.0020219278f, -0.0020269707f,
    -0.0020320268f, -0.0020370960f, -0.0020421784f, -0.0020472740f, -0.0020523828f, -0.0020575047f, -0.0020626398f, -0.0020677880f
};

static const SBRHuffEntry f_huff_env_1_5dB[121] = {
    {0x7ffe7, 19},    {0x7ffe8, 19},    {0xfffd2, 20},    {0xfffd3, 20},    {0xfffd4, 20},    {0xfffd5, 20},    {0xfffd6, 20},    {0xfffd7, 20},
    {0xfffd8, 20},    {0x7ffda, 19},    {0xfffd9, 20},    {0xfffda, 20},    {0xfffdb, 20},    {0xfffdc, 20},    {0x7ffdb, 19},    {0xfffdd, 20},
    {0x7ffdc, 19},    {0x7ffdd, 19},    {0xfffde, 20},    {0x3ffe4, 18},    {0xfffdf, 20},    {0xfffe0, 20},    {0xfffe1, 20},    {0x7ffde, 19},
    {0xfffe2, 20},    {0xfffe3, 20},    {0xfffe4, 20},    {0x7ffdf, 19},    {0xfffe5, 20},    {0x7ffe0, 19},    {0x3ffe8, 18},    {0x7ffe1, 19},
    {0x3ffe0, 18},    {0x3ffe9, 18},    {0x1ffef, 17},    {0x3ffe5, 18},    {0x1ffec, 17},    {0x1ffed, 17},    {0x1ffee, 17},    {0x0fff4, 16},
    {0x0fff3, 16},    {0x0fff0, 16},    {0x07ff7, 15},    {0x07ff6, 15},    {0x03ffa, 14},    {0x01ffa, 13},    {0x01ff9, 13},    {0x00ffa, 12},
    {0x00ff8, 12},    {0x007f9, 11},    {0x003fb, 10},    {0x001fc, 9},    {0x001fa, 9},    {0x000fb, 8},    {0x0007c, 7},    {0x0003c, 6},
    {0x0001c, 5},    {0x0000c, 4},    {0x00005, 3},    {0x00001, 2},    {0x00000, 2},    {0x00004, 3},    {0x0000d, 4},    {0x0001d, 5},
    {0x0003d, 6},    {0x000fa, 8},    {0x000fc, 8},    {0x001fb, 9},    {0x003fa, 10},    {0x007f8, 11},    {0x007fa, 11},    {0x007fb, 11},
    {0x00ff9, 12},    {0x00ffb, 12},    {0x01ff8, 13},    {0x01ffb, 13},    {0x03ff8, 14},    {0x03ff9, 14},    {0x0fff1, 16},    {0x0fff2, 16},
    {0x1ffea, 17},    {0x1ffeb, 17},    {0x3ffe1, 18},    {0x3ffe2, 18},    {0x3ffea, 18},    {0x3ffe3, 18},    {0x3ffe6, 18},    {0x3ffe7, 18},
    {0x3ffeb, 18},    {0xfffe6, 20},    {0x7ffe2, 19},    {0xfffe7, 20},    {0xfffe8, 20},    {0xfffe9, 20},    {0xfffea, 20},    {0xfffeb, 20},
    {0xfffec, 20},    {0x7ffe3, 19},    {0xfffed, 20},    {0xfffee, 20},    {0xfffef, 20},    {0xffff0, 20},    {0x7ffe4, 19},    {0xffff1, 20},
    {0x3ffec, 18},    {0xffff2, 20},    {0xffff3, 20},    {0x7ffe5, 19},    {0x7ffe6, 19},    {0xffff4, 20},    {0xffff5, 20},    {0xffff6, 20},
    {0xffff7, 20},    {0xffff8, 20},    {0xffff9, 20},    {0xffffa, 20},    {0xffffb, 20},    {0xffffc, 20},    {0xffffd, 20},    {0xffffe, 20},
    {0xfffff, 20}
};

static const SBRHuffEntry f_huff_env_3_0dB[63] = {
    {0x05ff7, 13},    {0x05ff8, 13},    {0x05ff9, 13},    {0x05ffa, 13},    {0x05ffb, 13},    {0x0bff8, 14},    {0x0bff9, 14},    {0x017fc, 11},
    {0x002fe, 8},    {0x0017e, 7},    {0x0002e, 4},    {0x0000a, 2},    {0x00004, 1},    {0x00016, 3},    {0x0005e, 5},    {0x000be, 6},
    {0x005fe, 9},    {0x02ffa, 12},    {0x05ff6, 13},    {0x0bffa, 14},    {0x0bffb, 14},    {0x0bffc, 14},    {0x0bffd, 14},    {0x0bffe, 14},
    {0x0bfff, 14},    {0x003fd, 10},    {0x001fd, 9},    {0x000fd, 8},    {0x0003e, 6},    {0x0000e, 4},    {0x00002, 2},    {0x00000, 1},
    {0x00006, 3},    {0x0001e, 5},    {0x000fc, 8},    {0x001fc, 9},    {0x003fc, 10},    {0x007fc, 11},    {0x00ffc, 12},    {0x01ffc, 13},
    {0x03ffa, 14},    {0x07ff9, 15},    {0x07ffa, 15},    {0x0fff8, 16},    {0x0fff9, 16},    {0x1fff6, 17},    {0x1fff7, 17},    {0x3fff5, 18},
    {0x3fff6, 18},    {0x3fff1, 18},    {0xffff8, 20},    {0x7fff1, 19},    {0x7fff2, 19},    {0x7fff3, 19},    {0xffff9, 20},    {0x7fff7, 19},
    {0x7fff4, 19},    {0xffffa, 20},    {0xffffb, 20},    {0xffffc, 20},    {0xffffd, 20},    {0xffffe, 20},    {0xfffff, 20}
};



static inline int clamp_int(int val, int min, int max)
{
    return av_clip(val, min, max);
}

static int compute_kx(int sampleRate, int bs_start_freq)
{
    int temp = (sampleRate < 32000) ? 3000 : (sampleRate < 64000) ? 4000 : 5000;
    int start_min = ((temp << 7) + (sampleRate >> 1)) / sampleRate;
    int row = (sampleRate <= 16000) ? 0 : (sampleRate <= 22050) ? 1 : (sampleRate <= 24000) ? 2 : (sampleRate <= 32000) ? 3 : (sampleRate <= 64000) ? 4 : 5;
    return clamp_int(start_min + sbr_offset[row][bs_start_freq & 15], 1, 63);
}

static inline int sbr_env_bands(const SBRInfo *sbr, const SbrFrameData *fd)
{
    return fd->freqRes ? sbr->numBands : sbr->numBandsLow;
}

static inline const int *sbr_env_edges(const SBRInfo *sbr, const SbrFrameData *fd)
{
    return fd->freqRes ? sbr->bandEdges : sbr->bandEdgesLow;
}

static void SbrQmfAnalysis(AACSBREncContext *sCtx, SBRInfo *sbr, const float *ovl_pos, float *energy, int kx, int k2)
{
    AVComplexFloat x[64], y[64];
    const float *p0 = qmf_c;

    for (int m = 0; m < 64; m++) {
        int n0 = 2 * m;
        float a = p0[0]   * ovl_pos[639 - n0]
                    + p0[128] * ovl_pos[511 - n0]
                    + p0[256] * ovl_pos[383 - n0]
                    + p0[384] * ovl_pos[255 - n0]
                    + p0[512] * ovl_pos[127 - n0];
        float b = p0[1]   * ovl_pos[638 - n0]
                    + p0[129] * ovl_pos[510 - n0]
                    + p0[257] * ovl_pos[382 - n0]
                    + p0[385] * ovl_pos[254 - n0]
                    + p0[513] * ovl_pos[126 - n0];
        x[m].re = a * sbr->twidCos[m] - b * sbr->twidSin[m];
        x[m].im = -(a * sbr->twidSin[m] + b * sbr->twidCos[m]);
        p0 += 2;
    }

    sCtx->fft_fn(sCtx->fft_ctx, y, x, sizeof(AVComplexFloat));

    for (int k = kx; k < k2; k++) {
        int kr = 63 - k;
        float Ar = 0.5f * (y[k].re + y[kr].re);
        float Ai = 0.5f * (y[kr].im - y[k].im);
        float Br = -0.5f * (y[k].im + y[kr].im);
        float Bi = 0.5f * (y[kr].re - y[k].re);
        float wr = sbr->oddCos[k];
        float wi = sbr->oddSin[k];
        float Sr = Ar + wr * Br - wi * Bi;
        float Si = Ai + wr * Bi + wi * Br;
        energy[k] += Sr * Sr + Si * Si;
    }
}

static inline int sbr_env_of_slot(int numEnvelopes, const int *envStart, int slot)
{
    int e = 0;
    while (e + 1 < numEnvelopes && slot >= envStart[e + 1]) e++;
    return e;
}

static void SbrAnalyze(AACSBREncContext *sCtx, SignalAnalysis *sa, float **fullPtrs, int nch, const int *isLfe, int numSamples, SBRInfo *sbr)
{
    int numSlots = numSamples / SBR_QMF_BANDS_64;
    sa->numSlots = numSlots;
    sa->sampled = numSlots;

    for (int ch = 0; ch < nch; ch++) {
        sa->ch[ch].transientSlot = -1;
        sa->ch[ch].transientStrength = 0.0f;
    }

    int transSlot = -1;
    for (int ch = 0; ch < nch; ch++) {
        if (isLfe && isLfe[ch]) continue;
        if (sa->ch[ch].transientSlot >= 0 && sa->ch[ch].transientStrength > 2.0f) {
            transSlot = sa->ch[ch].transientSlot;
            break;
        }
    }

    if (transSlot >= 0) {
        sa->frameClass = SBR_FRAME_CLASS_VARFIX;
        sa->numEnvelopes = 2;
        int border = clamp_int(transSlot, 2, numSlots - 2);
        sa->tEnv[0] = 0;
        sa->tEnv[1] = border;
        sa->tEnv[2] = numSlots;
        sa->bsPointer = 0;
    } else {
        sa->frameClass = SBR_FRAME_CLASS_FIXFIX;
        sa->numEnvelopes = sbr->numEnvFixFix;
        sa->bsPointer = 0;
        for (int i = 0; i <= sa->numEnvelopes; i++)
            sa->tEnv[i] = (i * numSlots) / sa->numEnvelopes;
    }

    for (int e = 0; e < sa->numEnvelopes; e++)
        sa->envSampled[e] = sa->tEnv[e + 1] - sa->tEnv[e];

    for (int ch = 0; ch < nch; ch++) {
        if (isLfe && isLfe[ch]) continue;
        for (int e = 0; e < sa->numEnvelopes; e++)
            memset(sa->bandE[ch][e], 0, SBR_QMF_BANDS_64 * sizeof(float));

        float ovlBuf[SBR_QMF_HIST_LEN];
        memcpy(ovlBuf, sbr->ch[ch].qmfOvl64, SBR_QMF_HIST_LEN * sizeof(float));

        for (int s = 0; s < numSlots; s++) {
            memmove(ovlBuf, ovlBuf + SBR_QMF_BANDS_64, (SBR_QMF_HIST_LEN - SBR_QMF_BANDS_64) * sizeof(float));
            memcpy(ovlBuf + SBR_QMF_HIST_LEN - SBR_QMF_BANDS_64, fullPtrs[ch] + s * SBR_QMF_BANDS_64, SBR_QMF_BANDS_64 * sizeof(float));

            int e = sbr_env_of_slot(sa->numEnvelopes, sa->tEnv, s);
            SbrQmfAnalysis(sCtx, sbr, ovlBuf, sa->bandE[ch][e], sbr->kx, sbr->k2);
        }
    }
}

static void sbr_adopt_envelope_grid(const SBRInfo *sbr, const SignalAnalysis *sa, SbrFrameData *fd)
{
    fd->numEnvelopes = sa->numEnvelopes;
    fd->frameClass   = sa->frameClass;
    fd->bsPointer    = sa->bsPointer;
    for (int i = 0; i <= sa->numEnvelopes; i++) fd->tEnv[i] = sa->tEnv[i];
    fd->eff_amp_res = (fd->numEnvelopes == 1) ? 0 : SBR_AMP_RES;
    fd->freqRes = sbr->bs_freq_res;
}

static void sbr_quantize_envelopes(const SBRInfo *sbr, int nch, const int *isLfe,
                                   const SignalAnalysis *sa, SbrFrameData *fd)
{
    int n_env = fd->numEnvelopes;
    int nb = sbr_env_bands(sbr, fd);
    const int *edges = sbr_env_edges(sbr, fd);

    for (int ch = 0; ch < nch; ch++) {
        if (isLfe && isLfe[ch]) continue;
        const float (*bandE)[SBR_QMF_BANDS_64] = sa->bandE[ch];
        int dlav = fd->eff_amp_res ? SBR_ENV_DELTA_LIMIT_HIRES : SBR_ENV_DELTA_LIMIT_LORES;
        for (int e = 0; e < n_env; e++) {
            int prevLevel = -1;
            for (int b = 0; b < nb; b++) {
                int k_lo = edges[b], k_hi = edges[b+1];
                int e_slots = sa->envSampled[e];
                if (e_slots < 1) e_slots = 1;
                float E = 0;
                for (int k = k_lo; k < k_hi; k++) E += bandE[e][k];
                E /= (float)(e_slots * (k_hi - k_lo));
                float factor = fd->eff_amp_res ? 1.0f : 2.0f;
                int level = lrintf(factor * (log2f(E + SBR_LOG_ENERGY_FLOOR) - SBR_ENV_LEVEL_LOG2_OFFSET));
                int raw_level = clamp_int(level, 0, 127);
                int max_val = fd->eff_amp_res ? 63 : 127;
                if (prevLevel < 0) {
                    raw_level = clamp_int(raw_level, 0, max_val);
                    fd->ch[ch].envData[e][b] = raw_level;
                    prevLevel = raw_level;
                } else {
                    int delta = clamp_int(raw_level - prevLevel, -dlav, dlav);
                    delta = clamp_int(prevLevel + delta, 0, max_val) - prevLevel;
                    fd->ch[ch].envData[e][b] = delta;
                    prevLevel += delta;
                }
                            }
        }
    }
}

static void SbrEncode(SBRInfo *sbr, float **timeDomain, int numChannels, const int *isLfe, int numSamples, SignalAnalysis *sa, SbrFrameData *fd)
{
    for (int ch = 0; ch < numChannels; ch++)
        if (!isLfe || !isLfe[ch])
            memcpy(sbr->ch[ch].qmfOvl64, timeDomain[ch] + numSamples - SBR_QMF_HIST_LEN, SBR_QMF_HIST_LEN * sizeof(float));

    sbr_adopt_envelope_grid(sbr, sa, fd);
    sbr_quantize_envelopes(sbr, numChannels, isLfe, sa, fd);
}

static void sbr_frame_silence(SbrFrameData *fd)
{
    fd->numEnvelopes = 1;
    fd->eff_amp_res  = 0;
    fd->frameClass   = SBR_FRAME_CLASS_FIXFIX;
    fd->tEnv[0]      = 0;
    fd->tEnv[1]      = 32;
    fd->bsPointer    = 0;
    fd->freqRes      = 1;
    for (int ch = 0; ch < 16; ch++) {
        for (int e = 0; e < SBR_MAX_ENVELOPES; e++) {
            for (int b = 0; b < SBR_MAX_BANDS; b++)
                fd->ch[ch].envData[e][b] = 0;
        }
    }
}

static void write_sbr_header(const SBRInfo *sbr, PutBitContext *pb)
{
    put_bits(pb, 1, SBR_AMP_RES);
    put_bits(pb, 4, sbr->bs_start_freq);
    put_bits(pb, 4, sbr->bs_stop_freq);
    put_bits(pb, 3, sbr->bs_xover_band);
    put_bits(pb, 2, 0);
    put_bits(pb, 1, 1);
    put_bits(pb, 1, 0);
    put_bits(pb, 2, sbr->bs_freq_scale);
    put_bits(pb, 1, sbr->bs_alter_scale);
    put_bits(pb, 2, 0);
}

static const int sbr_ceil_log2[] = { 0, 1, 2, 2, 3, 3 };

static void write_sbr_grid(const SBRInfo *sbr, const SbrFrameData *fd, PutBitContext *pb)
{
    int num_env = fd->numEnvelopes;
    put_bits(pb, 2, fd->frameClass);
    if (fd->frameClass == SBR_FRAME_CLASS_VARFIX) {
        put_bits(pb, 2, fd->tEnv[0]);
        put_bits(pb, 2, num_env - 1);
        for (int i = 0; i < num_env - 1; i++)
            put_bits(pb, 2, (fd->tEnv[i + 1] - fd->tEnv[i] - 2) / 2);
        int ptr_len = sbr_ceil_log2[num_env];
        put_bits(pb, ptr_len, fd->bsPointer);
        for (int i = 0; i < num_env; i++)
            put_bits(pb, 1, sbr->bs_freq_res);
    } else {
        put_bits(pb, 2, num_env > 1 ? 1 : 0);
        put_bits(pb, 1, sbr->bs_freq_res);
    }
}

static void write_sbr_dtdf(const SbrFrameData *fd, PutBitContext *pb)
{
    int n_q = fd->numEnvelopes > 1 ? 2 : 1;
    int len = fd->numEnvelopes + n_q;
    put_bits(pb, len, 0);
}

static void write_sbr_invf(PutBitContext *pb)
{
    put_bits(pb, 2, SBR_INVF_MODE);
}

static void put_huff(PutBitContext *pb, const SBRHuffEntry *table, int nsyms, int offset, int delta)
{
    int sym = clamp_int(delta + offset, 0, nsyms - 1);
    put_bits(pb, table[sym].len, table[sym].code);
}

static void write_sbr_envelope(const SBRInfo *sbr, const SbrFrameData *fd, PutBitContext *pb, int ch)
{
    const SBRHuffEntry *table = fd->eff_amp_res ? f_huff_env_3_0dB : f_huff_env_1_5dB;
    int nsyms = fd->eff_amp_res ? F_HUFF_ENV_3_0DB_NSYMS : F_HUFF_ENV_1_5DB_NSYMS;
    int offset = fd->eff_amp_res ? F_HUFF_ENV_3_0DB_OFFSET : F_HUFF_ENV_1_5DB_OFFSET;
    int first_bits = fd->eff_amp_res ? 6 : 7;
    int first_max = (1 << first_bits) - 1;
    int nb = sbr_env_bands(sbr, fd);

    for (int e = 0; e < fd->numEnvelopes; e++) {
        const int *env_ch = fd->ch[ch].envData[e];
        put_bits(pb, first_bits, clamp_int(env_ch[0], 0, first_max));
        for (int b = 1; b < nb; b++)
            put_huff(pb, table, nsyms, offset, env_ch[b]);
    }
}

static void write_sbr_noise(const SbrFrameData *fd, PutBitContext *pb)
{
    int n_q = fd->numEnvelopes > 1 ? 2 : 1;
    for (int ne = 0; ne < n_q; ne++)
        put_bits(pb, 5, SBR_NOISE_LEVEL_DEFAULT);
}

static void write_sbr_data(const SBRInfo *sbr, const SbrFrameData *fd, PutBitContext *pb, int elem_type, int ch0)
{
    int nch = (elem_type == TYPE_CPE) ? 2 : 1;

    if (elem_type == TYPE_CPE) {
        put_bits(pb, 1, 0); // bs_data_extra = 0
        put_bits(pb, 1, 0); // bs_coupling = 0
    } else {
        put_bits(pb, 1, 0); // bs_data_extra = 0
    }

    for (int ch = 0; ch < nch; ch++)
        write_sbr_grid(sbr, fd, pb);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_dtdf(fd, pb);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_invf(pb);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_envelope(sbr, fd, pb, ch0 + ch);
    for (int ch = 0; ch < nch; ch++)
        write_sbr_noise(fd, pb);

    for (int ch = 0; ch < nch; ch++)
        put_bits(pb, 1, 0); // bs_add_harmonic_flag = 0

    put_bits(pb, 1, 0); // bs_extended_data = 0
}

static void emit_sbr_payload(const SBRInfo *sbr, const SbrFrameData *fd, PutBitContext *pb, int elem_type, int ch0, int sendHeader)
{
    put_bits(pb, 4, 13); /* EXT_SBR_DATA (13 = 0xd) */
    put_bits(pb, 1, sendHeader & 1);
    if (sendHeader)
        write_sbr_header(sbr, pb);
    write_sbr_data(sbr, fd, pb, elem_type, ch0);
}

int ff_aac_sbr_enc_write_payload(AACSBREncContext *sCtx, PutBitContext *pb, int elem_type, int ch0)
{
    if (!sCtx || !sCtx->sbrInfo || !sCtx->sbrInfo->sbrPresent) return 0;

    SBRInfo *sbr = sCtx->sbrInfo;
    const SbrFrameData *fd = &sCtx->frameFIFO[(sCtx->frameHead + 1) % SBR_FRAME_FIFO];

    if (!sbr->headerDecided) {
        sbr->sendHeaderThisFrame = (sbr->frameCount++ % SBR_HEADER_PERIOD == 0);
        sbr->headerDecided = 1;
    }

    PutBitContext pb_tmp;
    uint8_t tmp_buf[1024];
    init_put_bits(&pb_tmp, tmp_buf, sizeof(tmp_buf));
    emit_sbr_payload(sbr, fd, &pb_tmp, elem_type, ch0, sbr->sendHeaderThisFrame);
    int payloadBits = put_bits_count(&pb_tmp);

    int fillBytes = (payloadBits + 7) / 8;
    int padBits = fillBytes * 8 - payloadBits;

    put_bits(pb, 3, TYPE_FIL);
    if (fillBytes < 15) {
        put_bits(pb, 4, fillBytes);
    } else {
        put_bits(pb, 4, 15);
        put_bits(pb, 8, fillBytes - 14);
    }

    emit_sbr_payload(sbr, fd, pb, elem_type, ch0, sbr->sendHeaderThisFrame);
    if (padBits > 0)
        put_bits(pb, padBits, 0);

    return 0;
}

static int cmp_int_fn(const void *a, const void *b) { return *(const int *)a - *(const int *)b; }
static int cmp_int16_fn(const void *a, const void *b) { return (int)(*(const int16_t *)a) - (int)(*(const int16_t *)b); }

static int compute_k2(int sampleRate, int bs_stop_freq)
{
    if (bs_stop_freq == 14 || bs_stop_freq == 15) return 64;
    int temp = (sampleRate < 32000) ? 3000 : (sampleRate < 64000) ? 4000 : 5000;
    int stop_min = ((temp << 8) + (sampleRate >> 1)) / sampleRate;
    int k2;
    if (bs_stop_freq < 14) {
        int16_t stop_dk[13];
        float prod = (float)stop_min;
        int prev = stop_min;
        float base = powf(64.0f / (float)stop_min, (float)(1.0f / 13.0f));
        for (int i = 0; i < 12; i++) {
            prod *= base;
            int present = (int)lrintf(prod);
            stop_dk[i] = (int16_t)(present - prev);
            prev = present;
        }
        stop_dk[12] = (int16_t)(64 - prev);
        qsort(stop_dk, 13, sizeof(int16_t), cmp_int16_fn);
        k2 = stop_min;
        for (int i = 0; i < bs_stop_freq; i++) k2 += stop_dk[i];
    } else {
        k2 = 64;
    }

    return k2;
}

static void build_freq_table(SBRInfo *sbr)
{
    int kx = sbr->kx, k2 = sbr->k2;
    int *edges = sbr->bandEdges;
    int n_master;

    int prev = kx;
    int bands_per_octave = 14 - 2 * sbr->bs_freq_scale;
    n_master = 2 * (int)(bands_per_octave * log2f((float)k2 / (float)kx) / 2.0f + 0.5f);
    n_master = clamp_int(n_master, 1, SBR_MAX_BANDS);
    for (int k = 0; k < n_master; k++) {
        int edge = (int)(kx * powf((float)k2 / (float)kx, (float)(k + 1) / (float)n_master) + 0.5f);
        edges[1 + k] = edge - prev;
        prev = edge;
    }
    qsort(edges + 1, n_master, sizeof(int), cmp_int_fn);
    edges[0] = kx;
    for (int k = 1; k <= n_master; k++) edges[k] += edges[k - 1];
    sbr->numBands = n_master;

    int n_low = (n_master + 1) >> 1;
    sbr->numBandsLow = n_low;
    int off = n_master & 1;
    for (int k = 0; k <= n_low; k++) {
        int idx = 2 * k - off;
        if (idx < 0) idx = 0;
        sbr->bandEdgesLow[k] = edges[idx];
    }
}

static SBRInfo *SbrInit(int channels, int sampleRate, int64_t bitRate)
{
    SBRInfo *sbrInfo = av_mallocz(sizeof(SBRInfo));
    if (!sbrInfo) return NULL;

    sbrInfo->numChannels = channels;
    sbrInfo->sampleRate  = sampleRate;
    sbrInfo->sbrPresent  = 1;

    sbrInfo->bs_freq_scale = 2;
    sbrInfo->bs_alter_scale = 1;
    sbrInfo->bs_start_freq = 5;
    sbrInfo->bs_stop_freq  = 0;
    sbrInfo->bs_xover_band = 0;
    sbrInfo->bs_freq_res   = 1;
    sbrInfo->numEnvFixFix   = 1;

    sbrInfo->kx = compute_kx(sampleRate, sbrInfo->bs_start_freq);
    sbrInfo->k2 = compute_k2(sampleRate, sbrInfo->bs_stop_freq);

    build_freq_table(sbrInfo);

    for (int m = 0; m < SBR_QMF_BANDS_64; m++) {
        sbrInfo->twidCos[m] = (float)cos(M_PI * m / 64.0);
        sbrInfo->twidSin[m] = (float)sin(M_PI * m / 64.0);
        sbrInfo->oddCos[m]  = (float)cos(M_PI * (2 * m + 1) / 128.0);
        sbrInfo->oddSin[m]  = (float)sin(M_PI * (2 * m + 1) / 128.0);
    }

    return sbrInfo;
}

AACSBREncContext *ff_aac_sbr_enc_init(int channels, int sampleRate, int64_t bitRate)
{
    AACSBREncContext *sCtx = av_mallocz(sizeof(AACSBREncContext));
    if (!sCtx) return NULL;

    sCtx->fullSampleRate    = sampleRate;
    sCtx->fullSampleRateIdx = 0;
    sCtx->sbrInfo           = SbrInit(channels, sampleRate, bitRate);
    if (!sCtx->sbrInfo) {
        av_free(sCtx);
        return NULL;
    }

    float scale = 1.0f;
    if (av_tx_init(&sCtx->fft_ctx, &sCtx->fft_fn, AV_TX_FLOAT_FFT, 0, 64, &scale, 0) < 0) {
        av_free(sCtx->sbrInfo);
        av_free(sCtx);
        return NULL;
    }

    for (int i = 0; i < SBR_FRAME_FIFO; i++)
        sbr_frame_silence(&sCtx->frameFIFO[i]);

    return sCtx;
}

void ff_aac_sbr_enc_close(AACSBREncContext *sCtx)
{
    if (!sCtx) return;
    if (sCtx->fft_ctx)
        av_tx_uninit(&sCtx->fft_ctx);
    if (sCtx->sbrInfo)
        av_free(sCtx->sbrInfo);
    av_free(sCtx);
}

void ff_aac_sbr_enc_process_frame(AACSBREncContext *sCtx, int numChannels, const int *isLfe,
                                  int frameLen, float **inputSamples, float **coreSamples)
{
    if (!sCtx) return;

    /* 31-tap anti-aliasing FIR halfband filter for 2:1 decimation */
    for (int ch = 0; ch < numChannels; ch++) {
        float *in = inputSamples[ch];
        float *out = coreSamples[ch];
        int max_idx = 2 * frameLen;
        for (int i = 0; i < frameLen; i++) {
            int idx = 2 * i;
            float sum = fir_halfband[0] * in[idx];
            for (int k = 1; k < 16; k++) {
                float s0 = (idx - k >= 0) ? in[idx - k] : in[0];
                float s1 = (idx + k < max_idx) ? in[idx + k] : in[max_idx - 1];
                sum += fir_halfband[k] * (s0 + s1);
            }
            out[i] = sum;
        }
    }

    sCtx->frameHead = (sCtx->frameHead + 1) % SBR_FRAME_FIFO;
    SbrFrameData *fd = &sCtx->frameFIFO[sCtx->frameHead];
    sCtx->sbrInfo->headerDecided = 0;

    SbrAnalyze(sCtx, &sCtx->signalAnalysis, inputSamples, numChannels, isLfe, 2 * frameLen, sCtx->sbrInfo);
    SbrEncode(sCtx->sbrInfo, inputSamples, numChannels, isLfe, 2 * frameLen, &sCtx->signalAnalysis, fd);
}
