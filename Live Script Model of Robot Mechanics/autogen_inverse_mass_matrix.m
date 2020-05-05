function Minv = autogen_inverse_mass_matrix(I_bw,I_com,I_fw,bw_com_distance,bw_com_init_angle,fw_com_distance,fw_com_init_angle,m_bw,m_com,m_fw,theta_bw,theta_fw,theta_com)
%AUTOGEN_INVERSE_MASS_MATRIX
%    MINV = AUTOGEN_INVERSE_MASS_MATRIX(I_BW,I_COM,I_FW,BW_COM_DISTANCE,BW_COM_INIT_ANGLE,FW_COM_DISTANCE,FW_COM_INIT_ANGLE,M_BW,M_COM,M_FW,THETA_BW,THETA_FW,THETA_COM)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    05-May-2020 15:57:49

t2 = bw_com_init_angle+theta_com;
t3 = fw_com_init_angle+theta_com;
t4 = bw_com_init_angle.*2.0;
t5 = bw_com_distance.^2;
t6 = fw_com_init_angle.*2.0;
t7 = fw_com_distance.^2;
t8 = m_com.^2;
t9 = theta_bw.*2.0;
t10 = theta_fw.*2.0;
t11 = theta_com.*2.0;
t12 = I_com.*m_bw.*2.0;
t13 = I_com.*m_com.*4.0;
t14 = I_com.*m_fw.*2.0;
t19 = I_com.*m_bw.*m_fw;
t24 = -bw_com_init_angle;
t25 = -fw_com_init_angle;
t15 = cos(t9);
t16 = cos(t10);
t17 = sin(t9);
t18 = sin(t10);
t20 = cos(t2);
t21 = cos(t3);
t22 = sin(t2);
t23 = sin(t3);
t26 = -t9;
t27 = -t10;
t28 = m_com.*t12;
t29 = m_com.*t14;
t30 = m_bw.*m_fw.*t7;
t31 = t3+t9;
t32 = t3+t10;
t33 = I_com.*t8.*2.0;
t34 = bw_com_init_angle+t25;
t35 = m_bw.*m_fw.*t5;
t36 = t2+t3;
t37 = m_com.*m_fw.*t7.*2.0;
t49 = m_bw.*m_com.*t5.*2.0;
t57 = m_bw.*t5.*t8;
t60 = m_fw.*t7.*t8;
t69 = fw_com_init_angle+t9+t24;
t70 = fw_com_init_angle+t10+t24;
t38 = t2+t26;
t39 = t2+t27;
t40 = cos(t36);
t41 = m_com.*t35;
t42 = m_com.*t30;
t43 = cos(t31);
t44 = cos(t32);
t45 = sin(t31);
t46 = sin(t32);
t47 = cos(t34);
t48 = sin(t34);
t50 = t9+t27;
t51 = bw_com_distance.*m_bw.*m_fw.*t20;
t52 = fw_com_distance.*m_bw.*m_fw.*t21;
t53 = bw_com_distance.*m_bw.*m_fw.*t22;
t54 = t12.*t15;
t55 = t14.*t16;
t56 = fw_com_distance.*m_bw.*m_fw.*t23;
t58 = t12.*t17;
t59 = t14.*t18;
t65 = bw_com_distance.*m_bw.*m_com.*t20.*2.0;
t66 = fw_com_distance.*m_com.*m_fw.*t21.*2.0;
t67 = bw_com_distance.*m_bw.*m_com.*t22.*2.0;
t68 = fw_com_distance.*m_com.*m_fw.*t23.*2.0;
t72 = t2+t32;
t73 = t16.*t35;
t77 = t15.*t30;
t80 = t18.*t35;
t82 = t17.*t30;
t85 = cos(t69);
t86 = cos(t70);
t87 = t3+t32;
t88 = t26+t36;
t90 = t26+t32;
t110 = t10+t69;
t120 = t31+t32;
t61 = cos(t38);
t62 = cos(t39);
t63 = sin(t38);
t64 = sin(t39);
t71 = cos(t50);
t74 = cos(t72);
t75 = -t65;
t76 = -t51;
t78 = -t52;
t79 = -t66;
t81 = sin(t72);
t83 = -t56;
t84 = -t68;
t89 = t10+t38;
t91 = fw_com_distance.*m_bw.*m_fw.*t43;
t92 = fw_com_distance.*m_bw.*m_fw.*t44;
t93 = fw_com_distance.*m_bw.*m_fw.*t45;
t94 = fw_com_distance.*m_bw.*m_fw.*t46;
t95 = cos(t87);
t96 = cos(t88);
t98 = sin(t88);
t99 = cos(t90);
t101 = sin(t90);
t102 = t2+t38;
t105 = fw_com_distance.*m_com.*m_fw.*t44.*2.0;
t108 = fw_com_distance.*m_com.*m_fw.*t46.*2.0;
t111 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t40;
t112 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t47;
t113 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t48;
t119 = cos(t110);
t121 = sin(t110);
t122 = t32+t38;
t125 = cos(t120);
t128 = sin(t120);
t132 = t26+t87;
t139 = t38+t39;
t140 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t85;
t141 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t86;
t97 = cos(t89);
t100 = sin(t89);
t103 = bw_com_distance.*m_bw.*m_fw.*t61;
t104 = bw_com_distance.*m_bw.*m_fw.*t62;
t106 = bw_com_distance.*m_bw.*m_fw.*t63;
t107 = bw_com_distance.*m_bw.*m_fw.*t64;
t109 = cos(t102);
t114 = bw_com_distance.*m_bw.*m_com.*t61.*2.0;
t115 = -t91;
t116 = bw_com_distance.*m_bw.*m_com.*t63.*2.0;
t117 = -t94;
t118 = -t108;
t123 = m_com.*t111;
t124 = t19.*t71;
t126 = cos(t122);
t130 = -t111;
t131 = t2+t89;
t134 = cos(t132);
t136 = sin(t132);
t137 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t74;
t138 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t81;
t144 = fw_com_distance.*m_bw.*m_fw.*t99;
t147 = fw_com_distance.*m_bw.*m_fw.*t101;
t148 = cos(t139);
t149 = sin(t139);
t150 = m_com.*t140;
t151 = m_com.*t141;
t152 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t96;
t153 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t98;
t154 = t30.*t95;
t158 = t37.*t95;
t159 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t119;
t160 = t42.*t95;
t161 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t121;
t165 = m_com.*m_fw.*t7.*t95.*-2.0;
t166 = t60.*t95;
t178 = (t30.*t125)./2.0;
t179 = (t30.*t128)./2.0;
t127 = -t104;
t129 = -t107;
t133 = cos(t131);
t135 = sin(t131);
t142 = -t123;
t143 = bw_com_distance.*m_bw.*m_fw.*t97;
t145 = bw_com_distance.*m_bw.*m_fw.*t100;
t146 = -t124;
t155 = -t138;
t157 = t35.*t109;
t162 = bw_com_distance.*fw_com_distance.*m_bw.*m_fw.*t126;
t163 = t49.*t109;
t164 = -t154;
t167 = t41.*t109;
t168 = m_bw.*m_com.*t5.*t109.*-2.0;
t171 = t57.*t109;
t172 = -t160;
t174 = -t166;
t180 = (t30.*t134)./2.0;
t182 = (t30.*t136)./2.0;
t183 = -t179;
t185 = (t35.*t149)./2.0;
t187 = (t35.*t148)./2.0;
t156 = -t145;
t169 = -t157;
t170 = m_com.*t162;
t173 = -t162;
t175 = -t167;
t177 = -t171;
t181 = (t35.*t135)./2.0;
t184 = (t35.*t133)./2.0;
t189 = t75+t76+t78+t79+t92+t103+t105+t114+t115+t127+t143+t144;
t176 = -t170;
t186 = -t181;
t188 = t53+t67+t83+t84+t93+t106+t116+t117+t118+t129+t147+t156;
t190 = t58+t59+t80+t82+t113+t153+t155+t161+t182+t183+t185+t186;
t191 = t19+t28+t29+t33+t41+t42+t57+t60+t142+t146+t150+t151+t172+t174+t175+t176+t177;
t192 = 1.0./t191;
t193 = (t188.*t192)./2.0;
t194 = t192.*(t51+t52+t65+t66+t91-t92-t103+t104-t105-t114-t143-t144).*(-1.0./2.0);
t196 = (t192.*(t51+t52+t65+t66+t91-t92-t103+t104-t105-t114-t143-t144))./2.0;
t197 = (t190.*t192)./2.0;
t195 = -t193;
t198 = -t197;
Minv = reshape([(t192.*(t12+t13+t14+t30+t35+t37+t49-t73-t77-t112+t130+t137+t140+t141+t152-t159+t164+t165+t168+t169+t173+t178+t180+t184+t187-I_com.*m_bw.*t15.*2.0-I_com.*m_fw.*t16.*2.0))./2.0,t198,t195,0.0,0.0,t198,(t192.*(t12+t13+t14+t30+t35+t37+t49+t54+t55+t73+t77+t112+t130-t137+t140+t141-t152+t159+t164+t165+t168+t169+t173-t178-t180-t184-t187))./2.0,t196,0.0,0.0,t195,t196,t192.*(t8+m_bw.*m_com+(m_bw.*m_fw)./2.0+m_com.*m_fw-(m_bw.*m_fw.*t71)./2.0).*2.0,0.0,0.0,0.0,0.0,0.0,1.0./I_bw,0.0,0.0,0.0,0.0,0.0,1.0./I_fw],[5,5]);
