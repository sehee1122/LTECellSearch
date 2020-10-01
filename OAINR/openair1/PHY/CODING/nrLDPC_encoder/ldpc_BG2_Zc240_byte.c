#include "PHY/sse_intrin.h"
// generated code for Zc=240, byte encoding
static inline void ldpc_BG2_Zc240_byte(uint8_t *c,uint8_t *d) {
  __m128i *csimd=(__m128i *)c,*dsimd=(__m128i *)d;

  __m128i *c2,*d2;

  int i2;
  for (i2=0; i2<15; i2++) {
     c2=&csimd[i2];
     d2=&dsimd[i2];

//row: 0
     d2[0]=_mm_xor_si128(c2[309],_mm_xor_si128(c2[4208],_mm_xor_si128(c2[3004],_mm_xor_si128(c2[938],_mm_xor_si128(c2[4531],_mm_xor_si128(c2[4544],_mm_xor_si128(c2[2164],_mm_xor_si128(c2[3370],_mm_xor_si128(c2[1591],_mm_xor_si128(c2[4300],_mm_xor_si128(c2[990],_mm_xor_si128(c2[2231],_mm_xor_si128(c2[1623],_mm_xor_si128(c2[4625],_mm_xor_si128(c2[3451],_mm_xor_si128(c2[4356],_mm_xor_si128(c2[2281],_mm_xor_si128(c2[186],_mm_xor_si128(c2[4689],_mm_xor_si128(c2[2311],_mm_xor_si128(c2[2322],_mm_xor_si128(c2[2940],_mm_xor_si128(c2[3549],_mm_xor_si128(c2[3542],_mm_xor_si128(c2[277],_mm_xor_si128(c2[2380],c2[3574]))))))))))))))))))))))))));

//row: 1
     d2[15]=_mm_xor_si128(c2[309],_mm_xor_si128(c2[609],_mm_xor_si128(c2[4508],_mm_xor_si128(c2[3304],_mm_xor_si128(c2[938],_mm_xor_si128(c2[1238],_mm_xor_si128(c2[32],_mm_xor_si128(c2[30],_mm_xor_si128(c2[2164],_mm_xor_si128(c2[2464],_mm_xor_si128(c2[3670],_mm_xor_si128(c2[1591],_mm_xor_si128(c2[1891],_mm_xor_si128(c2[4600],_mm_xor_si128(c2[1290],_mm_xor_si128(c2[2531],_mm_xor_si128(c2[1923],_mm_xor_si128(c2[126],_mm_xor_si128(c2[3751],_mm_xor_si128(c2[4656],_mm_xor_si128(c2[2281],_mm_xor_si128(c2[2581],_mm_xor_si128(c2[486],_mm_xor_si128(c2[190],_mm_xor_si128(c2[2611],_mm_xor_si128(c2[2622],_mm_xor_si128(c2[3240],_mm_xor_si128(c2[3849],_mm_xor_si128(c2[3842],_mm_xor_si128(c2[277],_mm_xor_si128(c2[577],_mm_xor_si128(c2[2680],c2[3874]))))))))))))))))))))))))))))))));

//row: 2
     d2[30]=_mm_xor_si128(c2[309],_mm_xor_si128(c2[609],_mm_xor_si128(c2[4208],_mm_xor_si128(c2[4508],_mm_xor_si128(c2[3304],_mm_xor_si128(c2[938],_mm_xor_si128(c2[1238],_mm_xor_si128(c2[32],_mm_xor_si128(c2[30],_mm_xor_si128(c2[2164],_mm_xor_si128(c2[2464],_mm_xor_si128(c2[3670],_mm_xor_si128(c2[1591],_mm_xor_si128(c2[1891],_mm_xor_si128(c2[4300],_mm_xor_si128(c2[4600],_mm_xor_si128(c2[1290],_mm_xor_si128(c2[2231],_mm_xor_si128(c2[2531],_mm_xor_si128(c2[1923],_mm_xor_si128(c2[126],_mm_xor_si128(c2[3451],_mm_xor_si128(c2[3751],_mm_xor_si128(c2[4656],_mm_xor_si128(c2[2281],_mm_xor_si128(c2[2581],_mm_xor_si128(c2[186],_mm_xor_si128(c2[486],_mm_xor_si128(c2[190],_mm_xor_si128(c2[2311],_mm_xor_si128(c2[2611],_mm_xor_si128(c2[2622],_mm_xor_si128(c2[2940],_mm_xor_si128(c2[3240],_mm_xor_si128(c2[3849],_mm_xor_si128(c2[3842],_mm_xor_si128(c2[277],_mm_xor_si128(c2[577],_mm_xor_si128(c2[2380],_mm_xor_si128(c2[2680],c2[3874]))))))))))))))))))))))))))))))))))))))));

//row: 3
     d2[45]=_mm_xor_si128(c2[609],_mm_xor_si128(c2[4508],_mm_xor_si128(c2[3304],_mm_xor_si128(c2[1238],_mm_xor_si128(c2[32],_mm_xor_si128(c2[4544],_mm_xor_si128(c2[30],_mm_xor_si128(c2[2464],_mm_xor_si128(c2[3370],_mm_xor_si128(c2[3670],_mm_xor_si128(c2[1891],_mm_xor_si128(c2[4600],_mm_xor_si128(c2[1290],_mm_xor_si128(c2[2531],_mm_xor_si128(c2[1923],_mm_xor_si128(c2[4625],_mm_xor_si128(c2[126],_mm_xor_si128(c2[3751],_mm_xor_si128(c2[4356],_mm_xor_si128(c2[4656],_mm_xor_si128(c2[2581],_mm_xor_si128(c2[486],_mm_xor_si128(c2[4689],_mm_xor_si128(c2[190],_mm_xor_si128(c2[2611],_mm_xor_si128(c2[2322],_mm_xor_si128(c2[2622],_mm_xor_si128(c2[3240],_mm_xor_si128(c2[3849],_mm_xor_si128(c2[3542],_mm_xor_si128(c2[3842],_mm_xor_si128(c2[577],_mm_xor_si128(c2[2680],_mm_xor_si128(c2[3574],c2[3874]))))))))))))))))))))))))))))))))));

//row: 4
     d2[60]=_mm_xor_si128(c2[1505],_mm_xor_si128(c2[1805],_mm_xor_si128(c2[905],_mm_xor_si128(c2[4500],_mm_xor_si128(c2[3901],_mm_xor_si128(c2[2134],_mm_xor_si128(c2[2434],_mm_xor_si128(c2[1243],_mm_xor_si128(c2[1241],_mm_xor_si128(c2[3638],_mm_xor_si128(c2[3360],_mm_xor_si128(c2[3660],_mm_xor_si128(c2[67],_mm_xor_si128(c2[2802],_mm_xor_si128(c2[3102],_mm_xor_si128(c2[997],_mm_xor_si128(c2[2501],_mm_xor_si128(c2[3727],_mm_xor_si128(c2[3134],_mm_xor_si128(c2[1322],_mm_xor_si128(c2[163],_mm_xor_si128(c2[1053],_mm_xor_si128(c2[3492],_mm_xor_si128(c2[3792],_mm_xor_si128(c2[1682],_mm_xor_si128(c2[1386],_mm_xor_si128(c2[3822],_mm_xor_si128(c2[3818],_mm_xor_si128(c2[4451],_mm_xor_si128(c2[246],_mm_xor_si128(c2[254],_mm_xor_si128(c2[1473],_mm_xor_si128(c2[1773],_mm_xor_si128(c2[3876],c2[271]))))))))))))))))))))))))))))))))));

//row: 5
     d2[75]=_mm_xor_si128(c2[7],_mm_xor_si128(c2[307],_mm_xor_si128(c2[4206],_mm_xor_si128(c2[3002],_mm_xor_si128(c2[2707],_mm_xor_si128(c2[636],_mm_xor_si128(c2[936],_mm_xor_si128(c2[4544],_mm_xor_si128(c2[4542],_mm_xor_si128(c2[2732],_mm_xor_si128(c2[1862],_mm_xor_si128(c2[2162],_mm_xor_si128(c2[3368],_mm_xor_si128(c2[1304],_mm_xor_si128(c2[1604],_mm_xor_si128(c2[4298],_mm_xor_si128(c2[1003],_mm_xor_si128(c2[2229],_mm_xor_si128(c2[1621],_mm_xor_si128(c2[4623],_mm_xor_si128(c2[3464],_mm_xor_si128(c2[4354],_mm_xor_si128(c2[2860],_mm_xor_si128(c2[1994],_mm_xor_si128(c2[2294],_mm_xor_si128(c2[184],_mm_xor_si128(c2[4687],_mm_xor_si128(c2[2324],_mm_xor_si128(c2[2320],_mm_xor_si128(c2[2615],_mm_xor_si128(c2[2953],_mm_xor_si128(c2[3547],_mm_xor_si128(c2[3540],_mm_xor_si128(c2[4774],_mm_xor_si128(c2[275],_mm_xor_si128(c2[2378],c2[3572]))))))))))))))))))))))))))))))))))));

//row: 6
     d2[90]=_mm_xor_si128(c2[2712],_mm_xor_si128(c2[3012],_mm_xor_si128(c2[2112],_mm_xor_si128(c2[908],_mm_xor_si128(c2[2708],_mm_xor_si128(c2[3341],_mm_xor_si128(c2[3641],_mm_xor_si128(c2[2435],_mm_xor_si128(c2[2433],_mm_xor_si128(c2[4567],_mm_xor_si128(c2[68],_mm_xor_si128(c2[1274],_mm_xor_si128(c2[3994],_mm_xor_si128(c2[4294],_mm_xor_si128(c2[2204],_mm_xor_si128(c2[3693],_mm_xor_si128(c2[120],_mm_xor_si128(c2[4326],_mm_xor_si128(c2[2529],_mm_xor_si128(c2[1355],_mm_xor_si128(c2[2260],_mm_xor_si128(c2[2554],_mm_xor_si128(c2[4684],_mm_xor_si128(c2[185],_mm_xor_si128(c2[2889],_mm_xor_si128(c2[2593],_mm_xor_si128(c2[215],_mm_xor_si128(c2[211],_mm_xor_si128(c2[3820],_mm_xor_si128(c2[844],_mm_xor_si128(c2[1453],_mm_xor_si128(c2[1446],_mm_xor_si128(c2[2680],_mm_xor_si128(c2[2980],_mm_xor_si128(c2[284],_mm_xor_si128(c2[1478],c2[3877]))))))))))))))))))))))))))))))))))));

//row: 7
     d2[105]=_mm_xor_si128(c2[3904],_mm_xor_si128(c2[4204],_mm_xor_si128(c2[3603],_mm_xor_si128(c2[3304],_mm_xor_si128(c2[2703],_mm_xor_si128(c2[2100],_mm_xor_si128(c2[1514],_mm_xor_si128(c2[4533],_mm_xor_si128(c2[34],_mm_xor_si128(c2[4232],_mm_xor_si128(c2[3642],_mm_xor_si128(c2[3041],_mm_xor_si128(c2[3640],_mm_xor_si128(c2[2739],_mm_xor_si128(c2[3039],_mm_xor_si128(c2[1835],_mm_xor_si128(c2[960],_mm_xor_si128(c2[1260],_mm_xor_si128(c2[674],_mm_xor_si128(c2[2466],_mm_xor_si128(c2[1565],_mm_xor_si128(c2[1865],_mm_xor_si128(c2[402],_mm_xor_si128(c2[702],_mm_xor_si128(c2[101],_mm_xor_si128(c2[3396],_mm_xor_si128(c2[2795],_mm_xor_si128(c2[101],_mm_xor_si128(c2[4299],_mm_xor_si128(c2[1327],_mm_xor_si128(c2[726],_mm_xor_si128(c2[734],_mm_xor_si128(c2[133],_mm_xor_si128(c2[3721],_mm_xor_si128(c2[2820],_mm_xor_si128(c2[3120],_mm_xor_si128(c2[2562],_mm_xor_si128(c2[1961],_mm_xor_si128(c2[3452],_mm_xor_si128(c2[2551],_mm_xor_si128(c2[2851],_mm_xor_si128(c2[3161],_mm_xor_si128(c2[1092],_mm_xor_si128(c2[1392],_mm_xor_si128(c2[791],_mm_xor_si128(c2[4081],_mm_xor_si128(c2[3480],_mm_xor_si128(c2[3785],_mm_xor_si128(c2[2884],_mm_xor_si128(c2[3184],_mm_xor_si128(c2[1422],_mm_xor_si128(c2[821],_mm_xor_si128(c2[1418],_mm_xor_si128(c2[517],_mm_xor_si128(c2[817],_mm_xor_si128(c2[2315],_mm_xor_si128(c2[2051],_mm_xor_si128(c2[1450],_mm_xor_si128(c2[2645],_mm_xor_si128(c2[2044],_mm_xor_si128(c2[2653],_mm_xor_si128(c2[1752],_mm_xor_si128(c2[2052],_mm_xor_si128(c2[3872],_mm_xor_si128(c2[4172],_mm_xor_si128(c2[3571],_mm_xor_si128(c2[1476],_mm_xor_si128(c2[875],_mm_xor_si128(c2[2670],_mm_xor_si128(c2[1784],c2[2084]))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

//row: 8
     d2[120]=_mm_xor_si128(c2[608],_mm_xor_si128(c2[908],_mm_xor_si128(c2[4507],_mm_xor_si128(c2[8],_mm_xor_si128(c2[3603],_mm_xor_si128(c2[11],_mm_xor_si128(c2[1237],_mm_xor_si128(c2[1537],_mm_xor_si128(c2[331],_mm_xor_si128(c2[344],_mm_xor_si128(c2[2740],_mm_xor_si128(c2[2463],_mm_xor_si128(c2[2763],_mm_xor_si128(c2[3969],_mm_xor_si128(c2[1890],_mm_xor_si128(c2[2190],_mm_xor_si128(c2[4599],_mm_xor_si128(c2[100],_mm_xor_si128(c2[1604],_mm_xor_si128(c2[2530],_mm_xor_si128(c2[2830],_mm_xor_si128(c2[2222],_mm_xor_si128(c2[425],_mm_xor_si128(c2[3750],_mm_xor_si128(c2[4050],_mm_xor_si128(c2[156],_mm_xor_si128(c2[2580],_mm_xor_si128(c2[2880],_mm_xor_si128(c2[485],_mm_xor_si128(c2[785],_mm_xor_si128(c2[489],_mm_xor_si128(c2[2610],_mm_xor_si128(c2[2910],_mm_xor_si128(c2[2921],_mm_xor_si128(c2[3254],_mm_xor_si128(c2[3554],_mm_xor_si128(c2[4148],_mm_xor_si128(c2[4141],_mm_xor_si128(c2[576],_mm_xor_si128(c2[876],_mm_xor_si128(c2[2679],_mm_xor_si128(c2[2979],c2[4173]))))))))))))))))))))))))))))))))))))))))));

//row: 9
     d2[135]=_mm_xor_si128(c2[312],_mm_xor_si128(c2[1513],_mm_xor_si128(c2[1813],_mm_xor_si128(c2[4211],_mm_xor_si128(c2[913],_mm_xor_si128(c2[3007],_mm_xor_si128(c2[4508],_mm_xor_si128(c2[941],_mm_xor_si128(c2[2142],_mm_xor_si128(c2[2442],_mm_xor_si128(c2[4534],_mm_xor_si128(c2[1236],_mm_xor_si128(c2[4532],_mm_xor_si128(c2[1234],_mm_xor_si128(c2[2140],_mm_xor_si128(c2[2167],_mm_xor_si128(c2[3368],_mm_xor_si128(c2[3668],_mm_xor_si128(c2[3373],_mm_xor_si128(c2[60],_mm_xor_si128(c2[1594],_mm_xor_si128(c2[2795],_mm_xor_si128(c2[3095],_mm_xor_si128(c2[4303],_mm_xor_si128(c2[990],_mm_xor_si128(c2[993],_mm_xor_si128(c2[2494],_mm_xor_si128(c2[2234],_mm_xor_si128(c2[3720],_mm_xor_si128(c2[1626],_mm_xor_si128(c2[3127],_mm_xor_si128(c2[4628],_mm_xor_si128(c2[1330],_mm_xor_si128(c2[3454],_mm_xor_si128(c2[156],_mm_xor_si128(c2[4359],_mm_xor_si128(c2[1061],_mm_xor_si128(c2[2284],_mm_xor_si128(c2[3485],_mm_xor_si128(c2[3785],_mm_xor_si128(c2[189],_mm_xor_si128(c2[1690],_mm_xor_si128(c2[4692],_mm_xor_si128(c2[1394],_mm_xor_si128(c2[2314],_mm_xor_si128(c2[3815],_mm_xor_si128(c2[2310],_mm_xor_si128(c2[3811],_mm_xor_si128(c2[2943],_mm_xor_si128(c2[4444],_mm_xor_si128(c2[3552],_mm_xor_si128(c2[254],_mm_xor_si128(c2[3545],_mm_xor_si128(c2[247],_mm_xor_si128(c2[4454],_mm_xor_si128(c2[280],_mm_xor_si128(c2[1481],_mm_xor_si128(c2[1781],_mm_xor_si128(c2[2383],_mm_xor_si128(c2[3884],_mm_xor_si128(c2[3577],c2[279])))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

//row: 10
     d2[150]=_mm_xor_si128(c2[1802],_mm_xor_si128(c2[2743],_mm_xor_si128(c2[193],c2[2624])));

//row: 11
     d2[165]=_mm_xor_si128(c2[1512],_mm_xor_si128(c2[612],_mm_xor_si128(c2[4207],_mm_xor_si128(c2[611],_mm_xor_si128(c2[2141],_mm_xor_si128(c2[935],_mm_xor_si128(c2[633],_mm_xor_si128(c2[933],_mm_xor_si128(c2[3367],_mm_xor_si128(c2[4273],_mm_xor_si128(c2[4573],_mm_xor_si128(c2[2794],_mm_xor_si128(c2[704],_mm_xor_si128(c2[2193],_mm_xor_si128(c2[3434],_mm_xor_si128(c2[2826],_mm_xor_si128(c2[729],_mm_xor_si128(c2[1029],_mm_xor_si128(c2[4654],_mm_xor_si128(c2[460],_mm_xor_si128(c2[760],_mm_xor_si128(c2[3484],_mm_xor_si128(c2[1389],_mm_xor_si128(c2[793],_mm_xor_si128(c2[1093],_mm_xor_si128(c2[3514],_mm_xor_si128(c2[3210],_mm_xor_si128(c2[3510],_mm_xor_si128(c2[2023],_mm_xor_si128(c2[4143],_mm_xor_si128(c2[4752],_mm_xor_si128(c2[4445],_mm_xor_si128(c2[4745],_mm_xor_si128(c2[1480],_mm_xor_si128(c2[3583],_mm_xor_si128(c2[4477],_mm_xor_si128(c2[4777],c2[2680])))))))))))))))))))))))))))))))))))));

//row: 12
     d2[180]=_mm_xor_si128(c2[2713],_mm_xor_si128(c2[3013],_mm_xor_si128(c2[2113],_mm_xor_si128(c2[909],_mm_xor_si128(c2[3342],_mm_xor_si128(c2[3642],_mm_xor_si128(c2[2436],_mm_xor_si128(c2[2434],_mm_xor_si128(c2[3637],_mm_xor_si128(c2[4568],_mm_xor_si128(c2[69],_mm_xor_si128(c2[1260],_mm_xor_si128(c2[3995],_mm_xor_si128(c2[4295],_mm_xor_si128(c2[2190],_mm_xor_si128(c2[3694],_mm_xor_si128(c2[3097],_mm_xor_si128(c2[121],_mm_xor_si128(c2[4327],_mm_xor_si128(c2[2530],_mm_xor_si128(c2[1356],_mm_xor_si128(c2[2261],_mm_xor_si128(c2[4685],_mm_xor_si128(c2[186],_mm_xor_si128(c2[2890],_mm_xor_si128(c2[2594],_mm_xor_si128(c2[216],_mm_xor_si128(c2[212],_mm_xor_si128(c2[845],_mm_xor_si128(c2[1454],_mm_xor_si128(c2[1447],_mm_xor_si128(c2[2681],_mm_xor_si128(c2[2981],_mm_xor_si128(c2[270],c2[1479]))))))))))))))))))))))))))))))))));

//row: 13
     d2[195]=_mm_xor_si128(c2[3907],_mm_xor_si128(c2[3007],_mm_xor_si128(c2[1803],_mm_xor_si128(c2[3],_mm_xor_si128(c2[4536],_mm_xor_si128(c2[3330],_mm_xor_si128(c2[3043],_mm_xor_si128(c2[3343],_mm_xor_si128(c2[2733],_mm_xor_si128(c2[963],_mm_xor_si128(c2[1869],_mm_xor_si128(c2[2169],_mm_xor_si128(c2[390],_mm_xor_si128(c2[3099],_mm_xor_si128(c2[4603],_mm_xor_si128(c2[1030],_mm_xor_si128(c2[422],_mm_xor_si128(c2[3124],_mm_xor_si128(c2[3424],_mm_xor_si128(c2[2250],_mm_xor_si128(c2[2855],_mm_xor_si128(c2[3155],_mm_xor_si128(c2[1080],_mm_xor_si128(c2[3784],_mm_xor_si128(c2[3188],_mm_xor_si128(c2[3488],_mm_xor_si128(c2[1110],_mm_xor_si128(c2[821],_mm_xor_si128(c2[1121],_mm_xor_si128(c2[1754],_mm_xor_si128(c2[2348],_mm_xor_si128(c2[2041],_mm_xor_si128(c2[2341],_mm_xor_si128(c2[2350],_mm_xor_si128(c2[3875],_mm_xor_si128(c2[1179],_mm_xor_si128(c2[2073],c2[2373])))))))))))))))))))))))))))))))))))));

//row: 14
     d2[210]=_mm_xor_si128(c2[3906],_mm_xor_si128(c2[4206],_mm_xor_si128(c2[1204],_mm_xor_si128(c2[3306],_mm_xor_si128(c2[304],_mm_xor_si128(c2[2102],_mm_xor_si128(c2[3914],_mm_xor_si128(c2[4535],_mm_xor_si128(c2[36],_mm_xor_si128(c2[1833],_mm_xor_si128(c2[3644],_mm_xor_si128(c2[642],_mm_xor_si128(c2[3642],_mm_xor_si128(c2[340],_mm_xor_si128(c2[640],_mm_xor_si128(c2[635],_mm_xor_si128(c2[962],_mm_xor_si128(c2[1262],_mm_xor_si128(c2[3074],_mm_xor_si128(c2[2468],_mm_xor_si128(c2[3965],_mm_xor_si128(c2[4265],_mm_xor_si128(c2[404],_mm_xor_si128(c2[704],_mm_xor_si128(c2[2501],_mm_xor_si128(c2[3398],_mm_xor_si128(c2[396],_mm_xor_si128(c2[103],_mm_xor_si128(c2[1900],_mm_xor_si128(c2[1329],_mm_xor_si128(c2[3126],_mm_xor_si128(c2[721],_mm_xor_si128(c2[2533],_mm_xor_si128(c2[3723],_mm_xor_si128(c2[421],_mm_xor_si128(c2[721],_mm_xor_si128(c2[2564],_mm_xor_si128(c2[4361],_mm_xor_si128(c2[3454],_mm_xor_si128(c2[152],_mm_xor_si128(c2[452],_mm_xor_si128(c2[1094],_mm_xor_si128(c2[1394],_mm_xor_si128(c2[3191],_mm_xor_si128(c2[4083],_mm_xor_si128(c2[1081],_mm_xor_si128(c2[3787],_mm_xor_si128(c2[485],_mm_xor_si128(c2[785],_mm_xor_si128(c2[2594],_mm_xor_si128(c2[1424],_mm_xor_si128(c2[3221],_mm_xor_si128(c2[1420],_mm_xor_si128(c2[2917],_mm_xor_si128(c2[3217],_mm_xor_si128(c2[2053],_mm_xor_si128(c2[3850],_mm_xor_si128(c2[2647],_mm_xor_si128(c2[4444],_mm_xor_si128(c2[2640],_mm_xor_si128(c2[4152],_mm_xor_si128(c2[4452],_mm_xor_si128(c2[3874],_mm_xor_si128(c2[4174],_mm_xor_si128(c2[1172],_mm_xor_si128(c2[1478],_mm_xor_si128(c2[3275],_mm_xor_si128(c2[2672],_mm_xor_si128(c2[4184],c2[4484])))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

//row: 15
     d2[225]=_mm_xor_si128(c2[3007],_mm_xor_si128(c2[4203],_mm_xor_si128(c2[4503],_mm_xor_si128(c2[2107],_mm_xor_si128(c2[3603],_mm_xor_si128(c2[903],_mm_xor_si128(c2[2414],_mm_xor_si128(c2[1802],_mm_xor_si128(c2[3636],_mm_xor_si128(c2[33],_mm_xor_si128(c2[333],_mm_xor_si128(c2[2430],_mm_xor_si128(c2[3941],_mm_xor_si128(c2[2443],_mm_xor_si128(c2[3939],_mm_xor_si128(c2[63],_mm_xor_si128(c2[1274],_mm_xor_si128(c2[1574],_mm_xor_si128(c2[1269],_mm_xor_si128(c2[2765],_mm_xor_si128(c2[4304],_mm_xor_si128(c2[701],_mm_xor_si128(c2[1001],_mm_xor_si128(c2[2199],_mm_xor_si128(c2[3695],_mm_xor_si128(c2[3703],_mm_xor_si128(c2[400],_mm_xor_si128(c2[130],_mm_xor_si128(c2[1626],_mm_xor_si128(c2[4321],_mm_xor_si128(c2[1033],_mm_xor_si128(c2[2524],_mm_xor_si128(c2[4020],_mm_xor_si128(c2[1350],_mm_xor_si128(c2[2861],_mm_xor_si128(c2[2255],_mm_xor_si128(c2[3751],_mm_xor_si128(c2[180],_mm_xor_si128(c2[1391],_mm_xor_si128(c2[1691],_mm_xor_si128(c2[2884],_mm_xor_si128(c2[4380],_mm_xor_si128(c2[2588],_mm_xor_si128(c2[4084],_mm_xor_si128(c2[210],_mm_xor_si128(c2[1721],_mm_xor_si128(c2[221],_mm_xor_si128(c2[1717],_mm_xor_si128(c2[854],_mm_xor_si128(c2[2350],_mm_xor_si128(c2[1448],_mm_xor_si128(c2[2944],_mm_xor_si128(c2[1441],_mm_xor_si128(c2[2952],_mm_xor_si128(c2[2975],_mm_xor_si128(c2[4171],_mm_xor_si128(c2[4471],_mm_xor_si128(c2[279],_mm_xor_si128(c2[1775],_mm_xor_si128(c2[1473],c2[2984]))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

//row: 16
     d2[240]=_mm_xor_si128(c2[5],_mm_xor_si128(c2[305],_mm_xor_si128(c2[3306],_mm_xor_si128(c2[3606],_mm_xor_si128(c2[4204],_mm_xor_si128(c2[2406],_mm_xor_si128(c2[2706],_mm_xor_si128(c2[3000],_mm_xor_si128(c2[1502],_mm_xor_si128(c2[634],_mm_xor_si128(c2[934],_mm_xor_si128(c2[3935],_mm_xor_si128(c2[4235],_mm_xor_si128(c2[4542],_mm_xor_si128(c2[3044],_mm_xor_si128(c2[4540],_mm_xor_si128(c2[3042],_mm_xor_si128(c2[3040],_mm_xor_si128(c2[1860],_mm_xor_si128(c2[2160],_mm_xor_si128(c2[362],_mm_xor_si128(c2[662],_mm_xor_si128(c2[3366],_mm_xor_si128(c2[1868],_mm_xor_si128(c2[1302],_mm_xor_si128(c2[1602],_mm_xor_si128(c2[4603],_mm_xor_si128(c2[104],_mm_xor_si128(c2[4296],_mm_xor_si128(c2[2498],_mm_xor_si128(c2[2798],_mm_xor_si128(c2[1001],_mm_xor_si128(c2[4302],_mm_xor_si128(c2[2227],_mm_xor_si128(c2[429],_mm_xor_si128(c2[729],_mm_xor_si128(c2[1634],_mm_xor_si128(c2[121],_mm_xor_si128(c2[4621],_mm_xor_si128(c2[3123],_mm_xor_si128(c2[3462],_mm_xor_si128(c2[1664],_mm_xor_si128(c2[1964],_mm_xor_si128(c2[4352],_mm_xor_si128(c2[2854],_mm_xor_si128(c2[1992],_mm_xor_si128(c2[2292],_mm_xor_si128(c2[494],_mm_xor_si128(c2[794],_mm_xor_si128(c2[182],_mm_xor_si128(c2[3183],_mm_xor_si128(c2[3483],_mm_xor_si128(c2[4685],_mm_xor_si128(c2[3187],_mm_xor_si128(c2[2322],_mm_xor_si128(c2[524],_mm_xor_si128(c2[824],_mm_xor_si128(c2[2318],_mm_xor_si128(c2[820],_mm_xor_si128(c2[2951],_mm_xor_si128(c2[1153],_mm_xor_si128(c2[1453],_mm_xor_si128(c2[3545],_mm_xor_si128(c2[2047],_mm_xor_si128(c2[3553],_mm_xor_si128(c2[2040],_mm_xor_si128(c2[4772],_mm_xor_si128(c2[273],_mm_xor_si128(c2[3274],_mm_xor_si128(c2[3574],_mm_xor_si128(c2[2376],_mm_xor_si128(c2[578],_mm_xor_si128(c2[878],_mm_xor_si128(c2[3570],_mm_xor_si128(c2[2072],c2[2371])))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

//row: 17
     d2[255]=_mm_xor_si128(c2[1206],_mm_xor_si128(c2[1506],_mm_xor_si128(c2[3307],_mm_xor_si128(c2[3607],_mm_xor_si128(c2[606],_mm_xor_si128(c2[2407],_mm_xor_si128(c2[2707],_mm_xor_si128(c2[4201],_mm_xor_si128(c2[1503],_mm_xor_si128(c2[1835],_mm_xor_si128(c2[2135],_mm_xor_si128(c2[3936],_mm_xor_si128(c2[4236],_mm_xor_si128(c2[944],_mm_xor_si128(c2[3030],_mm_xor_si128(c2[942],_mm_xor_si128(c2[3043],_mm_xor_si128(c2[1242],_mm_xor_si128(c2[3061],_mm_xor_si128(c2[3361],_mm_xor_si128(c2[363],_mm_xor_si128(c2[663],_mm_xor_si128(c2[4567],_mm_xor_si128(c2[1869],_mm_xor_si128(c2[2503],_mm_xor_si128(c2[2803],_mm_xor_si128(c2[4604],_mm_xor_si128(c2[90],_mm_xor_si128(c2[698],_mm_xor_si128(c2[2499],_mm_xor_si128(c2[2799],_mm_xor_si128(c2[2202],_mm_xor_si128(c2[4303],_mm_xor_si128(c2[3428],_mm_xor_si128(c2[430],_mm_xor_si128(c2[730],_mm_xor_si128(c2[2820],_mm_xor_si128(c2[122],_mm_xor_si128(c2[1023],_mm_xor_si128(c2[3124],_mm_xor_si128(c2[4663],_mm_xor_si128(c2[1650],_mm_xor_si128(c2[1950],_mm_xor_si128(c2[754],_mm_xor_si128(c2[2855],_mm_xor_si128(c2[4060],_mm_xor_si128(c2[3193],_mm_xor_si128(c2[3493],_mm_xor_si128(c2[480],_mm_xor_si128(c2[780],_mm_xor_si128(c2[1383],_mm_xor_si128(c2[3184],_mm_xor_si128(c2[3484],_mm_xor_si128(c2[1087],_mm_xor_si128(c2[3188],_mm_xor_si128(c2[3523],_mm_xor_si128(c2[510],_mm_xor_si128(c2[810],_mm_xor_si128(c2[3519],_mm_xor_si128(c2[821],_mm_xor_si128(c2[4152],_mm_xor_si128(c2[1154],_mm_xor_si128(c2[1454],_mm_xor_si128(c2[4746],_mm_xor_si128(c2[2048],_mm_xor_si128(c2[4754],_mm_xor_si128(c2[2041],_mm_xor_si128(c2[1174],_mm_xor_si128(c2[1474],_mm_xor_si128(c2[3275],_mm_xor_si128(c2[3575],_mm_xor_si128(c2[3577],_mm_xor_si128(c2[579],_mm_xor_si128(c2[879],_mm_xor_si128(c2[4771],c2[2073])))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

//row: 18
     d2[270]=_mm_xor_si128(c2[8],_mm_xor_si128(c2[1093],c2[823]));

//row: 19
     d2[285]=_mm_xor_si128(c2[2714],_mm_xor_si128(c2[1814],_mm_xor_si128(c2[610],_mm_xor_si128(c2[2102],_mm_xor_si128(c2[3343],_mm_xor_si128(c2[2137],_mm_xor_si128(c2[2135],_mm_xor_si128(c2[1235],_mm_xor_si128(c2[4569],_mm_xor_si128(c2[961],_mm_xor_si128(c2[3996],_mm_xor_si128(c2[1891],_mm_xor_si128(c2[3395],_mm_xor_si128(c2[4621],_mm_xor_si128(c2[4028],_mm_xor_si128(c2[2231],_mm_xor_si128(c2[1057],_mm_xor_si128(c2[1962],_mm_xor_si128(c2[4686],_mm_xor_si128(c2[2591],_mm_xor_si128(c2[2280],_mm_xor_si128(c2[4716],_mm_xor_si128(c2[4712],_mm_xor_si128(c2[546],_mm_xor_si128(c2[1140],_mm_xor_si128(c2[1148],_mm_xor_si128(c2[2682],_mm_xor_si128(c2[4770],c2[1180]))))))))))))))))))))))))))));

//row: 20
     d2[300]=_mm_xor_si128(c2[2109],_mm_xor_si128(c2[2409],_mm_xor_si128(c2[1509],_mm_xor_si128(c2[305],_mm_xor_si128(c2[2738],_mm_xor_si128(c2[3038],_mm_xor_si128(c2[1832],_mm_xor_si128(c2[1830],_mm_xor_si128(c2[1537],_mm_xor_si128(c2[3964],_mm_xor_si128(c2[4264],_mm_xor_si128(c2[671],_mm_xor_si128(c2[3391],_mm_xor_si128(c2[3691],_mm_xor_si128(c2[1601],_mm_xor_si128(c2[3090],_mm_xor_si128(c2[4331],_mm_xor_si128(c2[3723],_mm_xor_si128(c2[1926],_mm_xor_si128(c2[1034],_mm_xor_si128(c2[752],_mm_xor_si128(c2[1657],_mm_xor_si128(c2[4081],_mm_xor_si128(c2[4381],_mm_xor_si128(c2[2286],_mm_xor_si128(c2[1990],_mm_xor_si128(c2[4411],_mm_xor_si128(c2[4422],_mm_xor_si128(c2[241],_mm_xor_si128(c2[850],_mm_xor_si128(c2[843],_mm_xor_si128(c2[2077],_mm_xor_si128(c2[2377],_mm_xor_si128(c2[4480],c2[875]))))))))))))))))))))))))))))))))));

//row: 21
     d2[315]=_mm_xor_si128(c2[3909],_mm_xor_si128(c2[3009],_mm_xor_si128(c2[1805],_mm_xor_si128(c2[4214],_mm_xor_si128(c2[4538],_mm_xor_si128(c2[3332],_mm_xor_si128(c2[3030],_mm_xor_si128(c2[3330],_mm_xor_si128(c2[965],_mm_xor_si128(c2[1871],_mm_xor_si128(c2[2171],_mm_xor_si128(c2[392],_mm_xor_si128(c2[3101],_mm_xor_si128(c2[4590],_mm_xor_si128(c2[1032],_mm_xor_si128(c2[424],_mm_xor_si128(c2[3126],_mm_xor_si128(c2[3426],_mm_xor_si128(c2[2252],_mm_xor_si128(c2[2857],_mm_xor_si128(c2[3157],_mm_xor_si128(c2[1082],_mm_xor_si128(c2[3786],_mm_xor_si128(c2[3190],_mm_xor_si128(c2[3490],_mm_xor_si128(c2[1112],_mm_xor_si128(c2[823],_mm_xor_si128(c2[1123],_mm_xor_si128(c2[1741],_mm_xor_si128(c2[2350],_mm_xor_si128(c2[2043],_mm_xor_si128(c2[2343],_mm_xor_si128(c2[4140],_mm_xor_si128(c2[3877],_mm_xor_si128(c2[1181],_mm_xor_si128(c2[2075],c2[2375]))))))))))))))))))))))))))))))))))));

//row: 22
     d2[330]=_mm_xor_si128(c2[942],c2[3662]);

//row: 23
     d2[345]=_mm_xor_si128(c2[1500],_mm_xor_si128(c2[4295],c2[4656]));

//row: 24
     d2[360]=_mm_xor_si128(c2[335],_mm_xor_si128(c2[961],c2[878]));

//row: 25
     d2[375]=_mm_xor_si128(c2[604],c2[4655]);

//row: 26
     d2[390]=_mm_xor_si128(c2[4505],_mm_xor_si128(c2[6],_mm_xor_si128(c2[2414],_mm_xor_si128(c2[3605],_mm_xor_si128(c2[3905],_mm_xor_si128(c2[1514],_mm_xor_si128(c2[2701],_mm_xor_si128(c2[310],_mm_xor_si128(c2[335],_mm_xor_si128(c2[635],_mm_xor_si128(c2[3043],_mm_xor_si128(c2[4243],_mm_xor_si128(c2[1837],_mm_xor_si128(c2[4241],_mm_xor_si128(c2[1535],_mm_xor_si128(c2[1835],_mm_xor_si128(c2[1561],_mm_xor_si128(c2[1861],_mm_xor_si128(c2[4269],_mm_xor_si128(c2[3067],_mm_xor_si128(c2[361],_mm_xor_si128(c2[661],_mm_xor_si128(c2[669],_mm_xor_si128(c2[1003],_mm_xor_si128(c2[1303],_mm_xor_si128(c2[3696],_mm_xor_si128(c2[3697],_mm_xor_si128(c2[3997],_mm_xor_si128(c2[1591],_mm_xor_si128(c2[702],_mm_xor_si128(c2[3095],_mm_xor_si128(c2[1628],_mm_xor_si128(c2[1928],_mm_xor_si128(c2[4321],_mm_xor_si128(c2[1320],_mm_xor_si128(c2[3728],_mm_xor_si128(c2[4322],_mm_xor_si128(c2[1631],_mm_xor_si128(c2[1931],_mm_xor_si128(c2[2863],_mm_xor_si128(c2[3163],_mm_xor_si128(c2[757],_mm_xor_si128(c2[4053],_mm_xor_si128(c2[1362],_mm_xor_si128(c2[1662],_mm_xor_si128(c2[1693],_mm_xor_si128(c2[1993],_mm_xor_si128(c2[4386],_mm_xor_si128(c2[4382],_mm_xor_si128(c2[4682],_mm_xor_si128(c2[2291],_mm_xor_si128(c2[4386],_mm_xor_si128(c2[1680],_mm_xor_si128(c2[1980],_mm_xor_si128(c2[1723],_mm_xor_si128(c2[2023],_mm_xor_si128(c2[4416],_mm_xor_si128(c2[2019],_mm_xor_si128(c2[4112],_mm_xor_si128(c2[4412],_mm_xor_si128(c2[814],_mm_xor_si128(c2[2352],_mm_xor_si128(c2[2652],_mm_xor_si128(c2[246],_mm_xor_si128(c2[3246],_mm_xor_si128(c2[840],_mm_xor_si128(c2[3254],_mm_xor_si128(c2[548],_mm_xor_si128(c2[848],_mm_xor_si128(c2[4473],_mm_xor_si128(c2[4773],_mm_xor_si128(c2[2382],_mm_xor_si128(c2[1777],_mm_xor_si128(c2[2077],_mm_xor_si128(c2[4470],_mm_xor_si128(c2[3271],_mm_xor_si128(c2[580],c2[880])))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

//row: 27
     d2[405]=_mm_xor_si128(c2[4],c2[1691]);

//row: 28
     d2[420]=_mm_xor_si128(c2[2130],_mm_xor_si128(c2[69],c2[151]));

//row: 29
     d2[435]=_mm_xor_si128(c2[2701],c2[2823]);

//row: 30
     d2[450]=_mm_xor_si128(c2[1562],_mm_xor_si128(c2[3458],_mm_xor_si128(c2[4123],c2[571])));

//row: 31
     d2[465]=_mm_xor_si128(c2[12],_mm_xor_si128(c2[3911],_mm_xor_si128(c2[2707],_mm_xor_si128(c2[641],_mm_xor_si128(c2[4234],_mm_xor_si128(c2[3932],_mm_xor_si128(c2[4232],_mm_xor_si128(c2[2742],_mm_xor_si128(c2[1867],_mm_xor_si128(c2[2773],_mm_xor_si128(c2[3073],_mm_xor_si128(c2[1294],_mm_xor_si128(c2[4003],_mm_xor_si128(c2[693],_mm_xor_si128(c2[1934],_mm_xor_si128(c2[1326],_mm_xor_si128(c2[4028],_mm_xor_si128(c2[4328],_mm_xor_si128(c2[3154],_mm_xor_si128(c2[3759],_mm_xor_si128(c2[4059],_mm_xor_si128(c2[1984],_mm_xor_si128(c2[4688],_mm_xor_si128(c2[4092],_mm_xor_si128(c2[4392],_mm_xor_si128(c2[2014],_mm_xor_si128(c2[1710],_mm_xor_si128(c2[2010],_mm_xor_si128(c2[2643],_mm_xor_si128(c2[3252],_mm_xor_si128(c2[2945],_mm_xor_si128(c2[3245],_mm_xor_si128(c2[4779],_mm_xor_si128(c2[2083],_mm_xor_si128(c2[2977],c2[3277])))))))))))))))))))))))))))))))))));

//row: 32
     d2[480]=_mm_xor_si128(c2[1501],_mm_xor_si128(c2[1801],_mm_xor_si128(c2[601],_mm_xor_si128(c2[901],_mm_xor_si128(c2[4511],_mm_xor_si128(c2[911],_mm_xor_si128(c2[2130],_mm_xor_si128(c2[2430],_mm_xor_si128(c2[1239],_mm_xor_si128(c2[1237],_mm_xor_si128(c2[3371],_mm_xor_si128(c2[3671],_mm_xor_si128(c2[63],_mm_xor_si128(c2[2798],_mm_xor_si128(c2[3098],_mm_xor_si128(c2[693],_mm_xor_si128(c2[993],_mm_xor_si128(c2[2497],_mm_xor_si128(c2[3423],_mm_xor_si128(c2[3723],_mm_xor_si128(c2[3130],_mm_xor_si128(c2[1333],_mm_xor_si128(c2[4658],_mm_xor_si128(c2[159],_mm_xor_si128(c2[1064],_mm_xor_si128(c2[4350],_mm_xor_si128(c2[3488],_mm_xor_si128(c2[3788],_mm_xor_si128(c2[1393],_mm_xor_si128(c2[1693],_mm_xor_si128(c2[1382],_mm_xor_si128(c2[3518],_mm_xor_si128(c2[3818],_mm_xor_si128(c2[3814],_mm_xor_si128(c2[4147],_mm_xor_si128(c2[4447],_mm_xor_si128(c2[242],_mm_xor_si128(c2[250],_mm_xor_si128(c2[1484],_mm_xor_si128(c2[1784],_mm_xor_si128(c2[3572],_mm_xor_si128(c2[3872],c2[282]))))))))))))))))))))))))))))))))))))))))));

//row: 33
     d2[495]=_mm_xor_si128(c2[3300],_mm_xor_si128(c2[2400],_mm_xor_si128(c2[1211],_mm_xor_si128(c2[3944],_mm_xor_si128(c2[2738],_mm_xor_si128(c2[2736],_mm_xor_si128(c2[371],_mm_xor_si128(c2[1562],_mm_xor_si128(c2[4262],_mm_xor_si128(c2[4597],_mm_xor_si128(c2[2492],_mm_xor_si128(c2[3996],_mm_xor_si128(c2[423],_mm_xor_si128(c2[4629],_mm_xor_si128(c2[2832],_mm_xor_si128(c2[1658],_mm_xor_si128(c2[2563],_mm_xor_si128(c2[488],_mm_xor_si128(c2[3192],_mm_xor_si128(c2[2881],_mm_xor_si128(c2[518],_mm_xor_si128(c2[514],_mm_xor_si128(c2[810],_mm_xor_si128(c2[1147],_mm_xor_si128(c2[1741],_mm_xor_si128(c2[1749],_mm_xor_si128(c2[3283],_mm_xor_si128(c2[572],c2[1781]))))))))))))))))))))))))))));

//row: 34
     d2[510]=_mm_xor_si128(c2[2402],_mm_xor_si128(c2[2702],_mm_xor_si128(c2[4502],_mm_xor_si128(c2[1502],_mm_xor_si128(c2[1802],_mm_xor_si128(c2[3602],_mm_xor_si128(c2[613],_mm_xor_si128(c2[2413],_mm_xor_si128(c2[2411],_mm_xor_si128(c2[3031],_mm_xor_si128(c2[3331],_mm_xor_si128(c2[332],_mm_xor_si128(c2[2140],_mm_xor_si128(c2[3940],_mm_xor_si128(c2[2138],_mm_xor_si128(c2[3638],_mm_xor_si128(c2[3938],_mm_xor_si128(c2[4272],_mm_xor_si128(c2[4572],_mm_xor_si128(c2[1573],_mm_xor_si128(c2[964],_mm_xor_si128(c2[2464],_mm_xor_si128(c2[2764],_mm_xor_si128(c2[3699],_mm_xor_si128(c2[3999],_mm_xor_si128(c2[1000],_mm_xor_si128(c2[1594],_mm_xor_si128(c2[1894],_mm_xor_si128(c2[3694],_mm_xor_si128(c2[3398],_mm_xor_si128(c2[399],_mm_xor_si128(c2[4324],_mm_xor_si128(c2[4624],_mm_xor_si128(c2[1625],_mm_xor_si128(c2[4031],_mm_xor_si128(c2[1032],_mm_xor_si128(c2[2234],_mm_xor_si128(c2[3734],_mm_xor_si128(c2[4034],_mm_xor_si128(c2[760],_mm_xor_si128(c2[1060],_mm_xor_si128(c2[2860],_mm_xor_si128(c2[1950],_mm_xor_si128(c2[3450],_mm_xor_si128(c2[3750],_mm_xor_si128(c2[4389],_mm_xor_si128(c2[4689],_mm_xor_si128(c2[1690],_mm_xor_si128(c2[2294],_mm_xor_si128(c2[2594],_mm_xor_si128(c2[4394],_mm_xor_si128(c2[2283],_mm_xor_si128(c2[3783],_mm_xor_si128(c2[4083],_mm_xor_si128(c2[4419],_mm_xor_si128(c2[4719],_mm_xor_si128(c2[1720],_mm_xor_si128(c2[4715],_mm_xor_si128(c2[1416],_mm_xor_si128(c2[1716],_mm_xor_si128(c2[249],_mm_xor_si128(c2[549],_mm_xor_si128(c2[2349],_mm_xor_si128(c2[1143],_mm_xor_si128(c2[2943],_mm_xor_si128(c2[1151],_mm_xor_si128(c2[2651],_mm_xor_si128(c2[2951],_mm_xor_si128(c2[2370],_mm_xor_si128(c2[2670],_mm_xor_si128(c2[4470],_mm_xor_si128(c2[4473],_mm_xor_si128(c2[4773],_mm_xor_si128(c2[1774],_mm_xor_si128(c2[1183],_mm_xor_si128(c2[2683],c2[2983]))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

//row: 35
     d2[525]=_mm_xor_si128(c2[5],_mm_xor_si128(c2[305],_mm_xor_si128(c2[4204],_mm_xor_si128(c2[3000],_mm_xor_si128(c2[634],_mm_xor_si128(c2[934],_mm_xor_si128(c2[4542],_mm_xor_si128(c2[4540],_mm_xor_si128(c2[1535],_mm_xor_si128(c2[1860],_mm_xor_si128(c2[2160],_mm_xor_si128(c2[3366],_mm_xor_si128(c2[1302],_mm_xor_si128(c2[1602],_mm_xor_si128(c2[4296],_mm_xor_si128(c2[1001],_mm_xor_si128(c2[2227],_mm_xor_si128(c2[1634],_mm_xor_si128(c2[4621],_mm_xor_si128(c2[3462],_mm_xor_si128(c2[4352],_mm_xor_si128(c2[464],_mm_xor_si128(c2[1992],_mm_xor_si128(c2[2292],_mm_xor_si128(c2[182],_mm_xor_si128(c2[4685],_mm_xor_si128(c2[2322],_mm_xor_si128(c2[2318],_mm_xor_si128(c2[2951],_mm_xor_si128(c2[3545],_mm_xor_si128(c2[3553],_mm_xor_si128(c2[4772],_mm_xor_si128(c2[273],_mm_xor_si128(c2[2376],c2[3570]))))))))))))))))))))))))))))))))));

//row: 36
     d2[540]=_mm_xor_si128(c2[611],_mm_xor_si128(c2[67],c2[3216]));

//row: 37
     d2[555]=_mm_xor_si128(c2[3303],_mm_xor_si128(c2[1201],_mm_xor_si128(c2[2403],_mm_xor_si128(c2[301],_mm_xor_si128(c2[1214],_mm_xor_si128(c2[3911],_mm_xor_si128(c2[3932],_mm_xor_si128(c2[1830],_mm_xor_si128(c2[2741],_mm_xor_si128(c2[639],_mm_xor_si128(c2[2739],_mm_xor_si128(c2[337],_mm_xor_si128(c2[637],_mm_xor_si128(c2[374],_mm_xor_si128(c2[3071],_mm_xor_si128(c2[1565],_mm_xor_si128(c2[3962],_mm_xor_si128(c2[4262],_mm_xor_si128(c2[4600],_mm_xor_si128(c2[2498],_mm_xor_si128(c2[2495],_mm_xor_si128(c2[393],_mm_xor_si128(c2[3999],_mm_xor_si128(c2[1897],_mm_xor_si128(c2[426],_mm_xor_si128(c2[3123],_mm_xor_si128(c2[4632],_mm_xor_si128(c2[2530],_mm_xor_si128(c2[2820],_mm_xor_si128(c2[433],_mm_xor_si128(c2[733],_mm_xor_si128(c2[1661],_mm_xor_si128(c2[4358],_mm_xor_si128(c2[2551],_mm_xor_si128(c2[164],_mm_xor_si128(c2[464],_mm_xor_si128(c2[491],_mm_xor_si128(c2[3188],_mm_xor_si128(c2[3180],_mm_xor_si128(c2[1093],_mm_xor_si128(c2[2884],_mm_xor_si128(c2[482],_mm_xor_si128(c2[782],_mm_xor_si128(c2[521],_mm_xor_si128(c2[3218],_mm_xor_si128(c2[517],_mm_xor_si128(c2[2914],_mm_xor_si128(c2[3214],_mm_xor_si128(c2[1150],_mm_xor_si128(c2[3847],_mm_xor_si128(c2[1744],_mm_xor_si128(c2[4441],_mm_xor_si128(c2[1752],_mm_xor_si128(c2[4149],_mm_xor_si128(c2[4449],_mm_xor_si128(c2[3271],_mm_xor_si128(c2[1184],_mm_xor_si128(c2[575],_mm_xor_si128(c2[3272],_mm_xor_si128(c2[1784],_mm_xor_si128(c2[4181],c2[4481])))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

//row: 38
     d2[570]=_mm_xor_si128(c2[3000],_mm_xor_si128(c2[3300],_mm_xor_si128(c2[2400],_mm_xor_si128(c2[1211],_mm_xor_si128(c2[3644],_mm_xor_si128(c2[3944],_mm_xor_si128(c2[2738],_mm_xor_si128(c2[2736],_mm_xor_si128(c2[3032],_mm_xor_si128(c2[71],_mm_xor_si128(c2[371],_mm_xor_si128(c2[1562],_mm_xor_si128(c2[4297],_mm_xor_si128(c2[4597],_mm_xor_si128(c2[2492],_mm_xor_si128(c2[3996],_mm_xor_si128(c2[423],_mm_xor_si128(c2[4629],_mm_xor_si128(c2[2832],_mm_xor_si128(c2[1658],_mm_xor_si128(c2[2563],_mm_xor_si128(c2[2852],_mm_xor_si128(c2[188],_mm_xor_si128(c2[488],_mm_xor_si128(c2[3192],_mm_xor_si128(c2[2881],_mm_xor_si128(c2[518],_mm_xor_si128(c2[514],_mm_xor_si128(c2[1147],_mm_xor_si128(c2[1741],_mm_xor_si128(c2[1749],_mm_xor_si128(c2[2983],_mm_xor_si128(c2[3283],_mm_xor_si128(c2[572],c2[1781]))))))))))))))))))))))))))))))))));

//row: 39
     d2[585]=_mm_xor_si128(c2[4205],_mm_xor_si128(c2[4505],_mm_xor_si128(c2[3305],_mm_xor_si128(c2[3605],_mm_xor_si128(c2[2401],_mm_xor_si128(c2[2110],_mm_xor_si128(c2[35],_mm_xor_si128(c2[335],_mm_xor_si128(c2[3943],_mm_xor_si128(c2[3941],_mm_xor_si128(c2[1261],_mm_xor_si128(c2[1561],_mm_xor_si128(c2[2767],_mm_xor_si128(c2[703],_mm_xor_si128(c2[1003],_mm_xor_si128(c2[3397],_mm_xor_si128(c2[3697],_mm_xor_si128(c2[402],_mm_xor_si128(c2[1328],_mm_xor_si128(c2[1628],_mm_xor_si128(c2[1020],_mm_xor_si128(c2[4022],_mm_xor_si128(c2[2563],_mm_xor_si128(c2[2863],_mm_xor_si128(c2[3753],_mm_xor_si128(c2[1393],_mm_xor_si128(c2[1693],_mm_xor_si128(c2[4082],_mm_xor_si128(c2[4382],_mm_xor_si128(c2[4086],_mm_xor_si128(c2[1423],_mm_xor_si128(c2[1723],_mm_xor_si128(c2[1719],_mm_xor_si128(c2[4112],_mm_xor_si128(c2[2052],_mm_xor_si128(c2[2352],_mm_xor_si128(c2[2946],_mm_xor_si128(c2[2954],_mm_xor_si128(c2[4173],_mm_xor_si128(c2[4473],_mm_xor_si128(c2[1477],_mm_xor_si128(c2[1777],c2[2971]))))))))))))))))))))))))))))))))))))))))));

//row: 40
     d2[600]=_mm_xor_si128(c2[1213],_mm_xor_si128(c2[1805],_mm_xor_si128(c2[313],_mm_xor_si128(c2[905],_mm_xor_si128(c2[3908],_mm_xor_si128(c2[4500],_mm_xor_si128(c2[1842],_mm_xor_si128(c2[2434],_mm_xor_si128(c2[636],_mm_xor_si128(c2[1243],_mm_xor_si128(c2[634],_mm_xor_si128(c2[941],_mm_xor_si128(c2[1241],_mm_xor_si128(c2[3068],_mm_xor_si128(c2[3660],_mm_xor_si128(c2[4274],_mm_xor_si128(c2[4566],_mm_xor_si128(c2[67],_mm_xor_si128(c2[4264],_mm_xor_si128(c2[2495],_mm_xor_si128(c2[3102],_mm_xor_si128(c2[390],_mm_xor_si128(c2[997],_mm_xor_si128(c2[1894],_mm_xor_si128(c2[2501],_mm_xor_si128(c2[3120],_mm_xor_si128(c2[3727],_mm_xor_si128(c2[2527],_mm_xor_si128(c2[3134],_mm_xor_si128(c2[730],_mm_xor_si128(c2[1022],_mm_xor_si128(c2[1322],_mm_xor_si128(c2[4355],_mm_xor_si128(c2[163],_mm_xor_si128(c2[461],_mm_xor_si128(c2[753],_mm_xor_si128(c2[1053],_mm_xor_si128(c2[3185],_mm_xor_si128(c2[3792],_mm_xor_si128(c2[1090],_mm_xor_si128(c2[1682],_mm_xor_si128(c2[794],_mm_xor_si128(c2[1086],_mm_xor_si128(c2[1386],_mm_xor_si128(c2[3215],_mm_xor_si128(c2[3822],_mm_xor_si128(c2[3211],_mm_xor_si128(c2[3518],_mm_xor_si128(c2[3818],_mm_xor_si128(c2[3844],_mm_xor_si128(c2[4451],_mm_xor_si128(c2[4453],_mm_xor_si128(c2[246],_mm_xor_si128(c2[4446],_mm_xor_si128(c2[4753],_mm_xor_si128(c2[254],_mm_xor_si128(c2[1181],_mm_xor_si128(c2[1773],_mm_xor_si128(c2[3284],_mm_xor_si128(c2[3876],_mm_xor_si128(c2[4478],_mm_xor_si128(c2[4770],c2[271]))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))));

//row: 41
     d2[615]=_mm_xor_si128(c2[2108],_mm_xor_si128(c2[2408],_mm_xor_si128(c2[1508],_mm_xor_si128(c2[304],_mm_xor_si128(c2[2737],_mm_xor_si128(c2[3037],_mm_xor_si128(c2[1831],_mm_xor_si128(c2[1844],_mm_xor_si128(c2[1533],_mm_xor_si128(c2[3963],_mm_xor_si128(c2[4263],_mm_xor_si128(c2[670],_mm_xor_si128(c2[3390],_mm_xor_si128(c2[3690],_mm_xor_si128(c2[1600],_mm_xor_si128(c2[3104],_mm_xor_si128(c2[4330],_mm_xor_si128(c2[3722],_mm_xor_si128(c2[1925],_mm_xor_si128(c2[751],_mm_xor_si128(c2[1656],_mm_xor_si128(c2[2263],_mm_xor_si128(c2[4080],_mm_xor_si128(c2[4380],_mm_xor_si128(c2[2285],_mm_xor_si128(c2[1989],_mm_xor_si128(c2[4410],_mm_xor_si128(c2[4421],_mm_xor_si128(c2[240],_mm_xor_si128(c2[849],_mm_xor_si128(c2[842],_mm_xor_si128(c2[2076],_mm_xor_si128(c2[2376],_mm_xor_si128(c2[4479],c2[874]))))))))))))))))))))))))))))))))));
  }
}
