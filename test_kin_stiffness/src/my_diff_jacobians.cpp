#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

Eigen::Matrix<double, 7, 6> dJH_T_dq1(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {
    
    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t3 * t5;
    double t16 = t2 * t10;
    double t17 = t4 * t8;
    double t18 = t3 * t11;
    double t19 = t8 * t10;
    double t20 = t8 * t9 * t11;
    double t21 = t9 * t10 * t12;
    double t22 = t3 * 1050.0;
    double t26 = t2 * t5 * t9;
    double t27 = t4 * t5 * t9;
    double t29 = t2 * t9 * t11;
    double t30 = t5 * t8 * t9;
    double t31 = t4 * t9 * t11;
    double t32 = t6 * t9 * t10;
    double t33 = t3 * (9.0 / 25.0);
    double t34 = t5 * (39.0 / 50.0);
    double t35 = t7 * (59.0 / 50.0);
    double t43 = t2 * t9 * (9.0 / 25.0);
    double t44 = t8 * t9 * (9.0 / 25.0);
    double t23 = t3 * t14;
    double t24 = t3 * t16;
    double t25 = t3 * t17;
    double t28 = t3 * t19;
    double t36 = t15 * 1000.0;
    double t38 = -t27;
    double t41 = -t30;
    double t45 = t7 * t15 * 199.0;
    double t46 = t31 * 1000.0;
    double t47 = t7 * t31 * 199.0;
    double t48 = t6 * t13 * t18 * 199.0;
    double t49 = t13 * t21 * 199.0;
    double t50 = t31 * (39.0 / 50.0);
    double t53 = t15 + t31;
    double t54 = t6 * t13 * t27 * 199.0;
    double t58 = t34 - 39.0 / 50.0;
    double t59 = t35 - 59.0 / 50.0;
    double t37 = -t23;
    double t39 = -t28;
    double t51 = t16 + t25;
    double t52 = t17 + t24;
    double t57 = t18 + t38;
    double t60 = -t54;
    double t72 = t3 * t58;
    double t74 = t2 * t9 * t58;
    double t75 = t8 * t9 * t58;
    double t80 = t7 * t53 * 1.2596;
    double t89 = t53 * t59;
    double t55 = t14 + t39;
    double t56 = t19 + t37;
    double t61 = t5 * t51;
    double t62 = t6 * t52;
    double t63 = t11 * t51;
    double t64 = t12 * t52;
    double t68 = t6 * t57;
    double t71 = t12 * t57;
    double t83 = -t80;
    double t100 = t33 + t50 + t72 - 9.0 / 25.0;
    double t108 = t22 + t36 + t45 + t46 + t47 + t48 + t49 + t60;
    double t65 = t5 * t56;
    double t66 = t6 * t55;
    double t67 = -t62;
    double t69 = t11 * t56;
    double t70 = t12 * t55;
    double t76 = t63 * (39.0 / 50.0);
    double t79 = t20 + t61;
    double t81 = t41 + t63;
    double t82 = t21 + t68;
    double t94 = t7 * (t30 - t63) * (-1.2596);
    double t99 = t59 * (t30 - t63);
    double t73 = -t66;
    double t77 = -t76;
    double t78 = t69 * (39.0 / 50.0);
    double t84 = t26 + t69;
    double t85 = t6 * t79;
    double t86 = t12 * t79;
    double t90 = -t6 * (t29 - t65);
    double t91 = -t12 * (t29 - t65);
    double t92 = t13 * t82 * 0.0796;
    double t93 = t7 * t84 * 1.2596;
    double t95 = -t92;
    double t97 = t59 * t84;
    double t101 = t70 + t85;
    double t102 = t64 + t90;
    double t103 = t73 + t86;
    double t104 = t67 + t91;
    double t96 = -t93;
    double t105 = t13 * t101 * 0.0796;
    double t107 = t13 * t102 * 0.0796;
    double t109 = t83 + t89 + t95 + t100;
    double t106 = -t105;
    double t111 = t43 + t74 + t78 + t96 + t97 + t107;
    double t110 = t44 + t75 + t77 + t94 + t99 + t106;
    
    Matrix<double, 7, 6> dJ_T_dq1_mat;
    dJ_T_dq1_mat << t111, t8 * t108 * (-0.0004), t11 * t14 * (2.0 / 5.0) - t18 * t19 * (2.0 / 5.0) + t7 * t11 * t14 * 0.0796 + t12 * t13 * t16 * 0.0796 - t7 * t18 * t19 * 0.0796 + t12 * t13 * t25 * 0.0796 - t5 * t6 * t13 * t14 * 0.0796 + t6 * t13 * t15 * t19 * 0.0796, t20 * (2.0 / 5.0) + t5 * t16 * (2.0 / 5.0) + t7 * t20 * 0.0796 + t15 * t17 * (2.0 / 5.0) + t5 * t7 * t16 * 0.0796 + t7 * t15 * t17 * 0.0796 - t6 * t13 * t30 * 0.0796 + t6 * t11 * t13 * t16 * 0.0796 + t6 * t13 * t17 * t18 * 0.0796,
                t13 * (-t6 * t14 + t12 * t20 + t6 * t28 + t5 * t12 * t16 + t12 * t15 * t17) * 0.0796, t70 * (59.0 / 50.0) + t85 * (59.0 / 50.0) - t100 * (t62 + t12 * (t29 - t65)) + t109 * (t62 + t12 * (t29 - t65)) + t111 * (t32 - t71) - (t32 - t71) * (t43 + t74 + t78), 0.0, t110, (t2 * t108) / 2500.0, t11 * t17 * (2.0 / 5.0) + t16 * t18 * (2.0 / 5.0) + t7 * t11 * t17 * 0.0796 + t7 * t16 * t18 * 0.0796 + t12 * t13 * t19 * 0.0796 - t12 * t13 * t23 * 0.0796 - t5 * t6 * t13 * t17 * 0.0796 - t6 * t13 * t15 * t16 * 0.0796,
                t29 * (-2.0 / 5.0) + t5 * t19 * (2.0 / 5.0) - t14 * t15 * (2.0 / 5.0) - t7 * t29 * 0.0796 + t5 * t7 * t19 * 0.0796 - t7 * t14 * t15 * 0.0796 + t6 * t13 * t26 * 0.0796 + t6 * t11 * t13 * t19 * 0.0796 - t6 * t13 * t14 * t18 * 0.0796, t13 * (t6 * t17 + t6 * t24 + t12 * t29 - t5 * t12 * t19 + t12 * t14 * t15) * (-0.0796), t64 * (59.0 / 50.0) - t6 * (t29 - t65) * (59.0 / 50.0) + t110 * (t32 - t71) + t100 * (t66 - t86) - t109 * (t66 - t86) - (t32 - t71) * (t44 + t75 + t77), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                -t2, -t8 * t9, t55, t81, t103, -t13 * t101 - t7 * (t30 - t63), 0.0, -t8, t2 * t9, t52, t84, t104, t7 * t84 - t13 * t102, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    return dJ_T_dq1_mat;
}

Eigen::Matrix<double, 7, 6> dJH_T_dq2(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {
    

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t3 * t5;
    double t15 = t5 * t9;
    double t16 = t9 * t11;
    double t17 = t9 * t10 * t12;
    double t18 = t3 * 1050.0;
    double t19 = t9 * 1050.0;
    double t21 = t3 * t4 * t11;
    double t22 = t3 * t6 * t10;
    double t23 = t3 * t6 * t11;
    double t26 = t6 * t9 * t10;
    double t27 = t3 * t11 * t12;
    double t29 = t3 * t11 * 1000.0;
    double t31 = t10 * t11 * 1000.0;
    double t40 = t3 * t7 * t11 * 199.0;
    double t42 = t7 * t10 * t11 * 199.0;
    double t43 = t4 * t12 * t13 * 199.0;
    double t58 = t5 * t6 * t10 * t13 * 199.0;
    double t59 = t3 * t10 * t12 * t13 * 199.0;
    double t20 = t4 * t14;
    double t24 = t13 * t14;
    double t25 = t4 * t16;
    double t28 = t14 * 1000.0;
    double t30 = t15 * 1000.0;
    double t32 = t4 * t6 * t15;
    double t33 = t7 * t23;
    double t34 = t4 * t12 * t15;
    double t36 = t7 * t17;
    double t37 = -t27;
    double t38 = -t29;
    double t39 = t7 * t14 * 199.0;
    double t41 = t7 * t15 * 199.0;
    double t48 = -t40;
    double t49 = t21 * 1000.0;
    double t51 = -t43;
    double t53 = t7 * t21 * 199.0;
    double t57 = t13 * t23 * 199.0;
    double t60 = t6 * t13 * t16 * 199.0;
    double t62 = t13 * t17 * 199.0;
    double t66 = -t58;
    double t67 = -t59;
    double t35 = t13 * t25;
    double t44 = t7 * t32;
    double t45 = -t32;
    double t46 = -t33;
    double t47 = -t36;
    double t50 = t4 * t30;
    double t52 = t25 * 1000.0;
    double t54 = t4 * t41;
    double t55 = t6 * t24 * 199.0;
    double t56 = t7 * t25 * 199.0;
    double t61 = -t49;
    double t63 = t14 + t25;
    double t64 = t16 + t20;
    double t65 = -t53;
    double t68 = t6 * t13 * t20 * 199.0;
    double t69 = t13 * t32 * 199.0;
    double t73 = t26 + t34 + t37;
    double t74 = t31 + t42 + t51 + t66;
    double t70 = t6 * t35 * 199.0;
    double t71 = -t69;
    double t72 = t17 + t23 + t45;
    double t75 = t24 + t35 + t44 + t46 + t47;
    double t78 = t19 + t30 + t41 + t60 + t61 + t65 + t67 + t68;
    double t76 = t38 + t48 + t50 + t54 + t55 + t70;
    double t77 = t18 + t28 + t39 + t52 + t56 + t57 + t62 + t71;

    Matrix<double, 7, 6> dJ_T_dq2_mat;
    dJ_T_dq2_mat << t8 * t77 * (-0.0004), t2 * t78 * (-0.0004), t2 * t9 * t74 * (-0.0004), (t2 * t76) / 2500.0, t2 * t13 * t73 * 0.0796, t2 * t75 * (-0.0796),
                    0.0, (t2 * t77) / 2500.0, t8 * t78 * (-0.0004), t8 * t9 * t74 * (-0.0004), (t8 * t76) / 2500.0, t8 * t13 * t73 * 0.0796, t8 * t75 * (-0.0796),
                    0.0, 0.0, t3 * (-21.0 / 50.0) - t14 * (2.0 / 5.0) - t25 * (2.0 / 5.0) - t7 * t14 * 0.0796 - t13 * t17 * 0.0796 - t7 * t25 * 0.0796 - t13 * t23 * 0.0796 + t13 * t32 * 0.0796, t3 * t74 * (-0.0004), t16 * (2.0 / 5.0) + t20 * (2.0 / 5.0) + t7 * t16 * 0.0796 + t7 * t20 * 0.0796 - t6 * t13 * t15 * 0.0796 + t6 * t13 * t21 * 0.0796,
                    t13 * (t22 + t12 * t16 + t12 * t20) * 0.0796, t13 * t15 * 0.0796 - t13 * t21 * 0.0796 - t6 * t7 * t16 * 0.0796 - t6 * t7 * t20 * 0.0796 + t3 * t7 * t10 * t12 * 0.0796, 0.0, 0.0, 0.0, t2 * t3, -t2 * t9 * t10, t2 * t63, t2 * t26 - t12 * (t2 * t3 * t11 - t2 * t4 * t15),
                    t2 * t7 * t63 + t2 * t13 * t72, 0.0, 0.0, t3 * t8, -t8 * t9 * t10, t8 * t63, t8 * t26 - t12 * (t3 * t8 * t11 - t4 * t8 * t15), t7 * t8 * t63 + t8 * t13 * t72, 0.0, 0.0, -t9, -t3 * t10, -t15 + t21, t22 + t12 * t64, -t13 * (t6 * t64 - t3 * t10 * t12) - t7 * (t15 - t21);

    return dJ_T_dq2_mat;
}

Eigen::Matrix<double, 7, 6> dJH_T_dq3(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t4 * t6;
    double t16 = t2 * t10;
    double t17 = t4 * t8;
    double t18 = t8 * t10;
    double t19 = t5 * 1000.0;
    double t24 = t5 * t10 * t12;
    double t25 = t5 * t7 * 199.0;
    double t26 = t7 * (59.0 / 50.0);
    double t27 = t10 * t11 * 1000.0;
    double t31 = t7 * t10 * t11 * 199.0;
    double t32 = t4 * t12 * t13 * 199.0;
    double t33 = t6 * t11 * t13 * 199.0;
    double t35 = t5 * t6 * t10 * t13 * 199.0;
    double t20 = t3 * t14;
    double t21 = t3 * t16;
    double t22 = t3 * t17;
    double t23 = t3 * t18;
    double t30 = -t24;
    double t34 = -t32;
    double t38 = -t35;
    double t42 = t26 - 59.0 / 50.0;
    double t49 = t19 + t25 + t33;
    double t28 = -t20;
    double t29 = -t23;
    double t36 = t16 + t22;
    double t37 = t17 + t21;
    double t41 = t15 + t30;
    double t52 = t27 + t31 + t34 + t38;
    double t39 = t14 + t29;
    double t40 = t18 + t28;
    double t43 = t12 * t36;
    double t44 = t5 * t6 * t37;
    double t45 = t12 * t40;
    double t46 = t5 * t6 * t39;
    double t48 = -t46;
    double t50 = t43 + t48;

    Matrix<double, 7, 6> dJ_T_dq3_mat;
    dJ_T_dq3_mat << t11 * t39 * (-39.0 / 50.0) + t13 * t50 * 0.0796 + t7 * t11 * t39 * 1.2596 - t11 * t39 * t42, t2 * t9 * t52 * (-0.0004), t11 * t18 * (-2.0 / 5.0) + t11 * t20 * (2.0 / 5.0) - t7 * t11 * t18 * 0.0796 + t7 * t11 * t20 * 0.0796 + t12 * t13 * t17 * 0.0796 + t12 * t13 * t21 * 0.0796 + t5 * t6 * t13 * t18 * 0.0796 - t5 * t6 * t13 * t20 * 0.0796, (t37 * t49) / 2500.0, t13 * (t6 * t18 + t6 * t28 + t5 * t12 * t17 + t5 * t12 * t21) * 0.0796,
                    t7 * t12 * t18 * 0.0796 - t7 * t12 * t20 * 0.0796 - t11 * t13 * t17 * 0.0796 - t11 * t13 * t21 * 0.0796 - t5 * t7 * t8 * t15 * 0.0796 - t5 * t6 * t7 * t21 * 0.0796, 0.0, t11 * t37 * (-39.0 / 50.0) - t13 * (t44 - t45) * 0.0796 + t7 * t11 * t37 * 1.2596 - t11 * t37 * t42, t8 * t9 * t52 * (-0.0004),
                    t11 * t16 * (2.0 / 5.0) + t11 * t22 * (2.0 / 5.0) + t7 * t11 * t16 * 0.0796 - t12 * t13 * t14 * 0.0796 + t7 * t11 * t22 * 0.0796 + t12 * t13 * t23 * 0.0796 - t5 * t6 * t13 * t16 * 0.0796 - t3 * t5 * t8 * t13 * t15 * 0.0796, t39 * t49 * (-0.0004), t13 * (t6 * t16 + t3 * t8 * t15 + t5 * t12 * t14 + t5 * t12 * t29) * (-0.0796),
                    t7 * t12 * t16 * (-0.0796) + t11 * t13 * t14 * 0.0796 - t7 * t12 * t22 * 0.0796 - t11 * t13 * t23 * 0.0796 + t5 * t6 * t7 * t14 * 0.0796 - t5 * t6 * t7 * t23 * 0.0796, 0.0, 0.0, t3 * t52 * (-0.0004), t9 * (t4 * t11 * 1000.0 + t4 * t7 * t11 * 199.0 - t5 * t13 * t15 * 199.0 + t10 * t12 * t13 * 199.0) * (-0.0004), t9 * t10 * t49 * (-0.0004), t9 * t13 * t41 * 0.0796, t9 * (t4 * t7 * t12 + t10 * t11 * t13 + t5 * t6 * t7 * t10) * 0.0796,
                    0.0, 0.0, 0.0, 0.0, -t18 + t20, t11 * t37, t6 * t40 + t5 * t12 * t37, -t13 * (t44 - t45) + t7 * t11 * t37, 0.0, 0.0, 0.0, t36, -t11 * t39, -t6 * t36 - t5 * t12 * t39, -t13 * t50 - t7 * t11 * t39, 0.0, 0.0, 0.0, -t4 * t9, -t9 * t10 * t11, t9 * t41, t13 * (t4 * t9 * t12 + t5 * t6 * t9 * t10) - t7 * t9 * t10 * t11;

    return dJ_T_dq3_mat;
}

Eigen::Matrix<double, 7, 6> dJH_T_dq4(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t3 * t5;
    double t16 = t2 * t10;
    double t17 = t3 * t11;
    double t18 = t8 * t10;
    double t19 = t8 * t9 * t11;
    double t20 = t5 * 1000.0;
    double t22 = t3 * t4 * t8;
    double t23 = t2 * t5 * t9;
    double t24 = t4 * t5 * t9;
    double t26 = t2 * t9 * t11;
    double t27 = t5 * t8 * t9;
    double t28 = t4 * t9 * t11;
    double t29 = t5 * t7 * 199.0;
    double t39 = t6 * t11 * t13 * 199.0;
    double t21 = t3 * t14;
    double t25 = t3 * t18;
    double t30 = t17 * 1000.0;
    double t33 = -t26;
    double t34 = -t27;
    double t35 = t19 * (2.0 / 5.0);
    double t37 = t5 * t16 * (2.0 / 5.0);
    double t38 = t7 * t17 * 199.0;
    double t41 = t4 * t9 * t20;
    double t42 = t4 * t8 * t15 * (2.0 / 5.0);
    double t43 = t7 * t24 * 199.0;
    double t44 = t6 * t13 * t15 * 199.0;
    double t45 = t16 + t22;
    double t46 = t15 + t28;
    double t47 = t6 * t13 * t28 * 199.0;
    double t52 = t5 * t7 * t16 * 0.0796;
    double t53 = t7 * t19 * 0.0796;
    double t56 = t4 * t7 * t8 * t15 * 0.0796;
    double t57 = t6 * t13 * t27 * 0.0796;
    double t58 = t6 * t11 * t13 * t16 * 0.0796;
    double t60 = t4 * t6 * t8 * t13 * t17 * 0.0796;
    double t62 = t20 + t29 + t39;
    double t31 = -t21;
    double t32 = -t25;
    double t36 = -t30;
    double t40 = -t38;
    double t50 = t5 * t45;
    double t51 = t11 * t45;
    double t59 = -t57;
    double t48 = t14 + t32;
    double t49 = t18 + t31;
    double t65 = t36 + t40 + t41 + t43 + t44 + t47;
    double t66 = t35 + t37 + t42 + t52 + t53 + t56 + t58 + t59 + t60;
    double t54 = t5 * t49;
    double t55 = t11 * t49;
    double t63 = t23 + t55;
    double t64 = t33 + t54;

    Matrix<double, 7, 6> dJ_T_dq4_mat;
    dJ_T_dq4_mat << t66, (t2 * t65) / 2500.0, (t62 * (t4 * t8 + t3 * t16)) / 2500.0, -t48 * (t17 * (-2.0 / 5.0) + t24 * (2.0 / 5.0) - t7 * t17 * 0.0796 + t7 * t24 * 0.0796 + t6 * t13 * t15 * 0.0796 + t6 * t13 * t28 * 0.0796) - t9 * t10 * t66, t12 * t13 * (t23 + t11 * t18 - t14 * t17) * (-0.0796), t13 * t26 * 0.0796 - t5 * t13 * t18 * 0.0796 + t6 * t7 * t23 * 0.0796 + t13 * t14 * t15 * 0.0796 + t6 * t7 * t11 * t18 * 0.0796 - t6 * t7 * t14 * t17 * 0.0796, 0.0,
                    t26 * (-2.0 / 5.0) + t5 * t18 * (2.0 / 5.0) - t14 * t15 * (2.0 / 5.0) - t7 * t26 * 0.0796 + t5 * t7 * t18 * 0.0796 - t7 * t14 * t15 * 0.0796 + t6 * t13 * t23 * 0.0796 + t6 * t11 * t13 * t18 * 0.0796 - t6 * t13 * t14 * t17 * 0.0796, (t8 * t65) / 2500.0, t48 * t62 * (-0.0004),
                    t27 * (-2.0 / 5.0) + t11 * t16 * (2.0 / 5.0) - t7 * t27 * 0.0796 + t4 * t8 * t17 * (2.0 / 5.0) + t7 * t11 * t16 * 0.0796 - t6 * t13 * t19 * 0.0796 + t4 * t7 * t8 * t17 * 0.0796 - t5 * t6 * t13 * t16 * 0.0796 - t4 * t6 * t8 * t13 * t15 * 0.0796, t12 * t13 * (t34 + t11 * t16 + t4 * t8 * t17) * 0.0796, t13 * t19 * 0.0796 + t5 * t13 * t16 * 0.0796 + t6 * t7 * t27 * 0.0796 + t4 * t8 * t13 * t15 * 0.0796 - t6 * t7 * t11 * t16 * 0.0796 - t4 * t6 * t7 * t8 * t17 * 0.0796, 0.0, 0.0,
                    t4 * t15 * (2.0 / 5.0) + t9 * t11 * (2.0 / 5.0) + t4 * t7 * t15 * 0.0796 + t7 * t9 * t11 * 0.0796 - t5 * t6 * t9 * t13 * 0.0796 + t4 * t6 * t13 * t17 * 0.0796, t9 * t10 * t62 * (-0.0004), t15 * (-2.0 / 5.0) - t28 * (2.0 / 5.0) - t7 * t15 * 0.0796 - t7 * t28 * 0.0796 - t6 * t13 * t17 * 0.0796 + t6 * t13 * t24 * 0.0796, t12 * t13 * t46 * (-0.0796), t13 * t17 * 0.0796 - t13 * t24 * 0.0796 + t6 * t7 * t15 * 0.0796 + t6 * t7 * t28 * 0.0796, 0.0, 0.0,
                    0.0, 0.0, 0.0, t64, -t12 * t63, -t7 * (t26 - t54) + t6 * t13 * t63, 0.0, 0.0, 0.0, 0.0, -t19 - t50, -t12 * (t27 - t51), -t7 * (t19 + t50) + t6 * t13 * (t27 - t51), 0.0, 0.0, 0.0, 0.0, -t17 + t24, -t12 * t46, -t7 * (t17 - t24) + t6 * t13 * t46;

    return dJ_T_dq4_mat;
}

Eigen::Matrix<double, 7, 6> dJH_T_dq5(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {

    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t3 * t5;
    double t16 = t2 * t10;
    double t17 = t4 * t8;
    double t18 = t3 * t11;
    double t19 = t8 * t10;
    double t20 = t8 * t9 * t11;
    double t21 = t9 * t10 * t12;
    double t25 = t2 * t5 * t9;
    double t26 = t4 * t5 * t9;
    double t28 = t2 * t9 * t11;
    double t29 = t5 * t8 * t9;
    double t30 = t4 * t9 * t11;
    double t31 = t6 * t9 * t10;
    double t22 = t3 * t14;
    double t23 = t3 * t16;
    double t24 = t3 * t17;
    double t27 = t3 * t19;
    double t33 = -t26;
    double t36 = -t29;
    double t40 = t15 + t30;
    double t32 = -t22;
    double t34 = -t27;
    double t38 = t16 + t24;
    double t39 = t17 + t23;
    double t43 = t18 + t33;
    double t41 = t14 + t34;
    double t42 = t19 + t32;
    double t44 = t5 * t38;
    double t45 = t6 * t39;
    double t49 = t12 * t43;
    double t46 = t5 * t42;
    double t47 = t6 * t41;
    double t51 = t20 + t44;
    double t52 = t12 * t51;

    Matrix<double, 7, 6> dJ_T_dq5_mat;
    dJ_T_dq5_mat << t13 * (t47 - t52) * (-0.0796), t2 * t13 * (t31 - t49) * 0.0796, t3 * t13 * (t47 - t52) * (-0.0796) + t8 * t9 * t13 * (t31 - t49) * 0.0796, t12 * t13 * (t25 + t11 * t19 - t14 * t18) * (-0.0796), t13 * (t29 - t11 * t38) * (t31 - t49) * 0.0796 - t13 * t40 * (t47 - t52) * (-0.0796), t7 * (t6 * t17 + t6 * t23 + t12 * t28 - t5 * t12 * t19 + t12 * t14 * t15) * (-0.0796), 0.0, t13 * (t45 + t12 * (t28 - t46)) * (-0.0796),
                    t8 * t13 * (t31 - t49) * 0.0796, t3 * t13 * (t45 + t12 * (t28 - t46)) * (-0.0796) - t2 * t9 * t13 * (t31 - t49) * 0.0796, t12 * t13 * (t36 + t11 * t16 + t17 * t18) * 0.0796, t13 * (t25 + t11 * t42) * (t31 - t49) * (-0.0796) - t13 * t40 * (t45 + t12 * (t28 - t46)) * 0.0796, t7 * (-t6 * t14 + t12 * t20 + t6 * t27 + t5 * t12 * t16 + t12 * t15 * t17) * (-0.0796), 0.0, 0.0, t13 * (t3 * t6 * t10 + t4 * t12 * t15 + t9 * t11 * t12) * 0.0796, t9 * t13 * (t4 * t6 - t5 * t10 * t12) * 0.0796,
                    t12 * t13 * t40 * (-0.0796), t13 * (t21 + t6 * t18 + t6 * t33) * (-0.0796), t7 * (t31 - t12 * t18 + t12 * t26) * 0.0796, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, t12 * t39 - t6 * (t28 - t46), -t13 * (t45 + t12 * (t28 - t46)), 0.0, 0.0, 0.0, 0.0, 0.0, -t12 * t41 - t6 * t51, t13 * (t47 - t52), 0.0, 0.0, 0.0, 0.0, 0.0, -t21 - t6 * t43, t13 * (t31 - t49);

    return dJ_T_dq5_mat;
}

Eigen::Matrix<double, 7, 6> dJH_T_dq6(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {
    // Calculate all the required trigonometric values
    double t2 = cos(q1);
    double t3 = cos(q2);
    double t4 = cos(q3);
    double t5 = cos(q4);
    double t6 = cos(q5);
    double t7 = cos(q6);
    double t8 = sin(q1);
    double t9 = sin(q2);
    double t10 = sin(q3);
    double t11 = sin(q4);
    double t12 = sin(q5);
    double t13 = sin(q6);
    double t14 = t2 * t4;
    double t15 = t3 * t5;
    double t16 = t2 * t10;
    double t17 = t4 * t8;
    double t18 = t3 * t11;
    double t19 = t8 * t10;
    double t20 = t8 * t9 * t11;
    double t21 = t9 * t10 * t12;
    double t25 = t2 * t5 * t9;
    double t26 = t4 * t5 * t9;
    double t28 = t2 * t9 * t11;
    double t29 = t5 * t8 * t9;
    double t30 = t4 * t9 * t11;
    double t31 = t6 * t9 * t10;
    double t22 = t3 * t14;
    double t23 = t3 * t16;
    double t24 = t3 * t17;
    double t27 = t3 * t19;
    double t33 = -t26;
    double t40 = t15 + t30;
    double t32 = -t22;
    double t34 = -t27;
    double t38 = t16 + t24;
    double t39 = t17 + t23;
    double t43 = t18 + t33;
    double t53 = t13 * t40 * 0.0796;
    double t41 = t14 + t34;
    double t42 = t19 + t32;
    double t44 = t5 * t38;
    double t45 = t11 * t38;
    double t46 = t12 * t39;
    double t48 = t6 * t43;
    double t51 = t12 * t43;
    double t47 = t5 * t42;
    double t49 = t11 * t42;
    double t50 = t12 * t41;
    double t52 = t20 + t44;
    double t56 = t21 + t48;
    double t64 = t13 * (t29 - t45) * (-0.0796);
    double t57 = t25 + t49;
    double t58 = t6 * t52;
    double t61 = -t6 * (t28 - t47);
    double t63 = t7 * t56 * 0.0796;
    double t62 = t13 * t57 * 0.0796;
    double t65 = t50 + t58;
    double t66 = t46 + t61;
    double t67 = t7 * t65 * 0.0796;
    double t68 = t7 * t66 * 0.0796;
    double t70 = t64 + t67;
    double t71 = t62 + t68;

    Matrix<double, 7, 6> dJ_T_dq6_mat;
    dJ_T_dq6_mat << -t67 + t13 * (t29 - t45) * 0.0796, -t2 * (t53 - t63), -t3 * t70 - t8 * t9 * (t53 - t63), t41 * (t53 - t63) + t9 * t10 * t70, -t40 * t70 - (t29 - t45) * (t53 - t63), -t70 * (t31 - t51) - (t53 - t63) * (t6 * t41 - t12 * t52), 0.0,
                    -t71, -t8 * (t53 - t63), -t3 * t71 + t2 * t9 * (t53 - t63), t39 * (t53 - t63) + t9 * t10 * t71, -t40 * t71 + t57 * (t53 - t63), -(t6 * t39 + t12 * (t28 - t47)) * (t53 - t63) - t71 * (t31 - t51), 0.0, 0.0,
                    t5 * t9 * t13 * 0.0796 - t4 * t13 * t18 * 0.0796 + t3 * t7 * t10 * t12 * 0.0796 - t4 * t6 * t7 * t15 * 0.0796 - t6 * t7 * t9 * t11 * 0.0796, t9 * (t4 * t7 * t12 + t10 * t11 * t13 + t5 * t6 * t7 * t10) * 0.0796, t13 * t18 * 0.0796 - t13 * t26 * 0.0796 + t6 * t7 * t15 * 0.0796 + t6 * t7 * t30 * 0.0796, t7 * (t31 - t12 * t18 + t12 * t26) * 0.0796, t7 * t15 * (-0.0796) - t13 * t21 * 0.0796 - t7 * t30 * 0.0796 - t6 * t13 * t18 * 0.0796 + t6 * t13 * t26 * 0.0796, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -t13 * t57 - t7 * t66, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, t7 * t65 - t13 * (t29 - t45), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -t13 * t40 + t7 * t56;

    return dJ_T_dq6_mat;
}

Eigen::Matrix<double, 7, 6> dJH_T_dq7(double q1, double q2, double q3, double q4, double q5, double q6, double q7) {
    using namespace Eigen;

    Matrix<double, 7, 6> dJ_T_dq7_mat = Matrix<double, 7, 6>::Zero();

    return dJ_T_dq7_mat;
}
