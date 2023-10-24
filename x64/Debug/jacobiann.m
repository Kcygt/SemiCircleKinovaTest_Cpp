function J = jacobiann(q)
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    q6 = q(6);
    q7 = q(7);

    x0 = cos(q1);
    x1 = sin(q1);
    x2 = sin(q2);
    x3 = 0.4208 * x2;
    x4 = cos(q3);
    x5 = x0 * x4;
    x6 = 0.0128 * x5;
    x7 = cos(q4);
    x8 = x1 * x2;
    x9 = x7 * x8;
    x10 = cos(q2);
    x11 = sin(q3);
    x12 = x1 * x11;
    x13 = x10 * x12;
    x14 = sin(q4);
    x15 = x0 * x11;
    x16 = 0.3143 * x15;
    x17 = x1 * x4;
    x18 = x10 * x17;
    x19 = -x16 - 0.3143 * x18;
    x20 = cos(q6);
    x21 = -0.2874 * x9;
    x22 = x15 + x18;
    x23 = x14 * x22;
    x24 = sin(q6);
    x25 = x13 - x5;
    x26 = sin(q5);
    x27 = 0.2874 * x26;
    x28 = x14 * x2;
    x29 = x1 * x28;
    x30 = -x15 - x18;
    x31 = x29 + x30 * x7;
    x32 = cos(q5);
    x33 = 0.2874 * x32;
    x34 = x31 * x33;
    x35 = 0.4208 * x10;
    x36 = 0.0128 * x15;
    x37 = x10 * x7;
    x38 = 0.3143 * x37;
    x39 = 0.3143 * x5;
    x40 = 0.2874 * x37;
    x41 = 0.2874 * x28;
    x42 = x2 * x27;
    x43 = x10 * x14;
    x44 = x2 * x7;
    x45 = 0.0128 * x12;
    x46 = 0.3143 * x17;
    x47 = x10 * x15;
    x48 = x17 + x47;
    x49 = 0.2874 * x20;
    x50 = x14 * x49;
    x51 = x10 * x5;
    x52 = x12 - x51;
    x53 = x33 * (-x17 - x47);
    x54 = x0 * x28;
    x55 = 0.3143 * x12;
    x56 = 0.3143 * x51;
    x57 = x52 * x7;
    x58 = x0 * x2;
    x59 = x58 * x7;
    x60 = -x12 + x51;
    x61 = x14 * x60;
    x62 = -x59 - x61;
    x63 = x24 * x33;
    x64 = -x54 + x60 * x7;
    x65 = x26 * x64;
    x66 = 0.2874 * x59;
    x67 = 0.2874 * x61;
    x68 = x26 * x48;
    x69 = 0.2874 * x68;
    x70 = x32 * x64;
    x71 = -x13 + x5;
    x72 = x25 * x33;
    x73 = x22 * x7;
    x74 = x14 * x30;
    x75 = x26 * x71;
    x76 = 0.3143 * x44;
    x77 = x10 * x11;
    x78 = 0.3143 * x43;
    x79 = 0.2874 * x44;
    x80 = 0.2874 * x43;
    x81 = x2 * x4;
    x82 = x11 * x28;
    x83 = x11 * x2;
    x84 = x33 * x83;
    x85 = x28 * x4;
    x86 = -x37 + x85;
    x87 = x4 * x44;
    x88 = -x43 - x87;
    x89 = x23 + x9;
    x90 = x29 - x73;
    x91 = x43 + x87;

    J = [-0.0118 * x0 - x1 * x3 + 0.0128 * x13 + x14 * x19 + x20 * (x21 - 0.2874 * x23) + x24 * (x25 * x27 + x34) - x6 - 0.3143 * x9, ...
        x0* x35 + x0 * x38 + x2 * x36 + x20 * (x0 * x40 - x41 * x5) + x24 * (x15 * x42 + x33 * (-x0 * x43 - x44 * x5)) - x28 * x39,...
        -x10 * x6 + x14 * (-x10 * x16 - x46) + x24 * (x27 * x52 + x53 * x7) + x45 - x48 * x50,...
        x20* (-0.2874 * x54 - 0.2874 * x57) - 0.3143 * x54 + x62 * x63 + x7 * (-x55 + x56),...
        x24* (x53 - 0.2874 * x65),...
        x20* (-x69 + 0.2874 * x70) - x24 * (x66 + x67),...
        0;...
        -x0 * x3 + 0.0118 * x1 + x14 * (x55 - x56) + 0.0128 * x17 + x20 * (-x66 - x67) + x24 * (x33 * (x54 + x57) + x69) + 0.0128 * x47 - 0.3143 * x59,...
        -x1 * x35 - x1 * x38 - x2 * x45 + x20 * (-x1 * x40 + x17 * x41) + x24 * (-x12 * x42 + x33 * (x1 * x43 + x17 * x44)) + x28 * x46,...
        x14* (0.3143 * x13 - x39) + 0.0128 * x18 + x24 * (x22 * x27 + x7 * x72) + x36 - x50 * x71,...
        x19* x7 + x20 * (0.2874 * x29 - 0.2874 * x73) + 0.3143 * x29 + x63 * (-x74 + x9),...
        x24* (-x27 * x31 + x72),...
        x20* (x34 - 0.2874 * x75) - x24 * (x21 + 0.2874 * x74),...
        0;...
        0,...
        x20* (-x4 * x80 - x79) + x24 * (x27 * x77 + x33 * (x28 - x37 * x4)) - x3 - x4 * x78 - x76 + 0.0128 * x77,...
        x24* (x27 * x81 + x7 * x84) + x49 * x82 + 0.0128 * x81 + 0.3143 * x82,...
        x20* (-x4 * x79 - x80) - x4 * x76 + x63 * x86 - x78,...
        x24* (-x27 * x88 + x84),...
        x20* (x27 * x83 + x33 * x88) - x24 * (x40 - 0.2874 * x85),...
        0;...
        0,...
        x1,...
        -x58,...
        x48,...
        x62,...
        x32* x48 + x65,...
        x20* x62 - x24 * (-x68 + x70);...
        0,...
        x0,...
        x8,...
        x71,...
        x89,...
        x26* x90 + x32 * x71,...
        x20* x89 - x24 * (x32 * x90 - x75);...
        -1,...
        0,...
        -x10,...
        -x83,...
        x86,...
        -x26 * x91 - x32 * x83,...
        x20* x86 - x24 * (x26 * x83 - x32 * x91);
    ];
end
