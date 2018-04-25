const unsigned char firCoeffsRaw[] = {
0xa7, 0x2b, 0x18, 0x3a, 0x11, 0x77, 0x14, 0x3a, 0xca, 0xd8, 0xe, 0x3a, 0x6f, 0xf9, 0x5, 0x3a, 0xb, 0xe8, 0xef, 0x39, 0x4b, 0xbb, 0xc4, 0x39, 0x7c, 0xae, 0x84, 0x39, 0x1f, 0x4a, 0xa5, 0x38, 0x81, 0x5d, 0x28, 0xb9, 0xbb, 0xb8, 0xfa, 0xb9, 0xa4, 0x6a, 0x68, 0xba, 0x63, 0x11, 0xb7, 0xba, 0xdb, 0x31, 0x4, 0xbb, 0x2b, 0x71, 0x34, 0xbb, 0xad, 0x5e, 0x6c, 0xbb, 0xe1, 0xe1, 0x95, 0xbb, 0xd2, 0x8, 0xb9, 0xbb, 0x67, 0x2d, 0xdf, 0xbb, 0xee, 0xd3, 0x3, 0xbc, 0xda, 0xcf, 0x18, 0xbc, 0xff, 0x6, 0x2e, 0xbc, 0xde, 0xe0, 0x42, 0xbc, 0x1e, 0xb3, 0x56, 0xbc, 0xbf, 0xc5, 0x68, 0xbc, 0x1c, 0x58, 0x78, 0xbc, 0x58, 0x53, 0x82, 0xbc, 0xb2, 0x78, 0x86, 0xbc, 0x19, 0x41, 0x88, 0xbc, 0xf8, 0x59, 0x87, 0xbc, 0x72, 0x7c, 0x83, 0xbc, 0x15, 0xe1, 0x78, 0xbc, 0xdf, 0x1f, 0x64, 0xbc, 0xc1, 0x90, 0x48, 0xbc, 0xd, 0x3b, 0x26, 0xbc, 0xc6, 0xa8, 0xfa, 0xbb, 0x87, 0x83, 0x9c, 0xbb, 0x71, 0xb9, 0xcc, 0xba, 0x1f, 0x52, 0xff, 0x3a, 0x93, 0xaf, 0xba, 0x3b, 0xbf, 0xa1, 0x1d, 0x3c, 0xd1, 0x99, 0x5f, 0x3c, 0x25, 0xfd, 0x90, 0x3c, 0x29, 0xb7, 0xb1, 0x3c, 0x73, 0x4d, 0xd1, 0x3c, 0x13, 0x14, 0xef, 0x3c, 0xb7, 0x32, 0x5, 0x3d, 0x6e, 0x53, 0x11, 0x3d, 0x86, 0xa6, 0x1b, 0x3d, 0x3, 0xf0, 0x23, 0x3d, 0x54, 0xff, 0x29, 0x3d, 0xb7, 0xb0, 0x2d, 0x3d, 0x4d, 0xee, 0x2e, 0x3d, 0xb7, 0xb0, 0x2d, 0x3d, 0x54, 0xff, 0x29, 0x3d, 0x3, 0xf0, 0x23, 0x3d, 0x86, 0xa6, 0x1b, 0x3d, 0x6e, 0x53, 0x11, 0x3d, 0xb7, 0x32, 0x5, 0x3d, 0x13, 0x14, 0xef, 0x3c, 0x73, 0x4d, 0xd1, 0x3c, 0x29, 0xb7, 0xb1, 0x3c, 0x25, 0xfd, 0x90, 0x3c, 0xd1, 0x99, 0x5f, 0x3c, 0xbf, 0xa1, 0x1d, 0x3c, 0x93, 0xaf, 0xba, 0x3b, 0x1f, 0x52, 0xff, 0x3a, 0x71, 0xb9, 0xcc, 0xba, 0x87, 0x83, 0x9c, 0xbb, 0xc6, 0xa8, 0xfa, 0xbb, 0xd, 0x3b, 0x26, 0xbc, 0xc1, 0x90, 0x48, 0xbc, 0xdf, 0x1f, 0x64, 0xbc, 0x15, 0xe1, 0x78, 0xbc, 0x72, 0x7c, 0x83, 0xbc, 0xf8, 0x59, 0x87, 0xbc, 0x19, 0x41, 0x88, 0xbc, 0xb2, 0x78, 0x86, 0xbc, 0x58, 0x53, 0x82, 0xbc, 0x1c, 0x58, 0x78, 0xbc, 0xbf, 0xc5, 0x68, 0xbc, 0x1e, 0xb3, 0x56, 0xbc, 0xde, 0xe0, 0x42, 0xbc, 0xff, 0x6, 0x2e, 0xbc, 0xda, 0xcf, 0x18, 0xbc, 0xee, 0xd3, 0x3, 0xbc, 0x67, 0x2d, 0xdf, 0xbb, 0xd2, 0x8, 0xb9, 0xbb, 0xe1, 0xe1, 0x95, 0xbb, 0xad, 0x5e, 0x6c, 0xbb, 0x2b, 0x71, 0x34, 0xbb, 0xdb, 0x31, 0x4, 0xbb, 0x63, 0x11, 0xb7, 0xba, 0xa4, 0x6a, 0x68, 0xba, 0xbb, 0xb8, 0xfa, 0xb9, 0x81, 0x5d, 0x28, 0xb9, 0x1f, 0x4a, 0xa5, 0x38, 0x7c, 0xae, 0x84, 0x39, 0x4b, 0xbb, 0xc4, 0x39, 0xb, 0xe8, 0xef, 0x39, 0x6f, 0xf9, 0x5, 0x3a, 0xca, 0xd8, 0xe, 0x3a, 0x11, 0x77, 0x14, 0x3a, 0xa7, 0x2b, 0x18, 0x3a
};

const float *const firCoeffs = (float *)firCoeffsRaw;