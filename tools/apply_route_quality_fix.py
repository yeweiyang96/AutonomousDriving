#!/usr/bin/env python3
"""Validate and install the captured Python route-quality fix on Windows.

Only adrive/navmesh.py and the new adrive/road_corridor.py can be installed.
Navigation data, DLLs, the navigation manifest and its identity are never edited.
The inactive helper is written first; navmesh.py is the final activation point.
"""
from __future__ import annotations

import argparse
import base64
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import stat
import sys
import tempfile
import types
import uuid
import zlib

VERSION = 'route-quality-v1'
OLD_NAV_SHA = 'df99282b34214e01d2b63fc451d7d8abad85edb1b864b3f766d3b94f29cd748a'
DLL_SHA = 'fc5191c9d63c4e4b3c917ef4e522ffd6f85c01520df881583f881acbe30d56ed'
BUNDLE_SHA = '9d5065b341ec1063f083e319807c12d3f47eb2c67d2f8a11f23a20b2d6b1fd18'
MAP0_SHA = '11f29fcf6940331baab4221832dd3c60cafd7b8946d3f7503bbfaffc66695226'
BRIDGE_TILE_SHA = '74dd28d4d559518bca84b678ac3fb3ccd5ba294c8ab65bebcede927acc46df1c'
SOURCE_HASHES = {
    'models': '5e13b795e98fd67a26d659b99f73ba6c9c30b8c52e178106460c2fd5e7bb6167',
    'data': 'd25c9a508cf69cfed7d8584ce5eeb9324ed42ab1368d2d32fddbe895a0931385',
    'planner': '5b4794ad8815acc9201d357aaea7ea9d1355380abed2312481ae069445eb7123',
}
SETTINGS = dict(cell_size_yards=.2666666, agent_radius_yards=.5333332,
                agent_height_yards=1.5999996, walkable_climb_yards=.5333332,
                max_slope_degrees=50, offmesh_connections=False,
                water=False, dynamic_obstacles=False)
CLIENT = dict(version='1.60.1.70658', product='wowf',
              build_key='165410aca770316ea0f88100bb4830c2',
              build_info_sha256='a33feb82421b4e8a67cedbec8f3bf19b5b2cdaa2c14de5d357dd71d7e013d3b3')
SHORT_A = (-10065.478503779887, -1507.212281364077, 28.680936813354492)
SHORT_B = (-10079.181030588508, -1468.9011349807895, 28.680936813354492)
LONG_A = (-9486.727746196266, 66.20478324520349, 56.24121856689453)
LONG_B = (-10550.815788443762, -1196.5663646349744, 27.7370548248291)
FILE_LIMIT = 32 * 1024 * 1024
ANY = object()

# Frozen production modules; every decoded byte is checked before use.
PAYLOAD_READY = True
PAYLOAD = {'navmesh': {'sha256': '64efa0d0f8f069632644c2db11bff6dd3a72a0a437f545f23c38bd28fbbd325d',
             'bytes': 23034,
             'b85': 'c-p-@Ym*$sk>B@Mw8e++mU>&es|SiXhOjJT2Vo%*LfGe<o9O82so807PxrXHXLm<?8-Z;e;w&D=0k$z<AY{XVEeXtl4Z{2xW*?vY7cMias-M-fBm3?)6q<fiWo2dMt1`3t*s)`m-PGS?THklq!zf98PxGSjw544>{{!cdi{E+jdzv45!KBX;Egdp#95c^Pm=^MGF>{kB)ZDPIC8?WE5_@Us%3u)qA+xk0^Vf&zvb*KRj9&ZRvKIw`pZL%xbT|EVcm|h~#1GdAd=PnVpgqFUXcB8<*V_Pimm@$a44Fr5(})_byCZfoVeux5ZS65XK?oM28jZ#Pv==Zfip71vQkbBdx?0Gf{nEe<f<)^9bS?1%7N)@z!McD*=na{-!EiRQi`Z*86$FKYMnNA?abuYOdhGY@rDMmAEe+ymq&d!Dl1^ggIGR5iM{x?!!YHNbB}=>wOd@65Y2f!{8}Vf1hVDAzg{=Tjmii+m+sJ!M!~d^GAzPBY)9Dyi#Z9ye^3l*uh5%pQ{utIqUXR>#D6gYLUZrd_9smva*g&~q0lmiyU4R}FFU`6QVd48gaxpcVs4HGxd-T#p=ZQ!E<)Wqi@XFOok2u$^JT97zqCNxCOZWj034mOT<0!VY$0I*XE$wPFNny56u=Novjp!IhZr|~u7^sfR%KHz1m!d6aed71o($dANSFc=sLThU~OB(#GwlqTr9viV_=nSG5=C50tzW>Qz_rH90cIV!~jSu%;ySw+o?Y(#I&F;PQ&7WVGJ#%k%`!9dHck|%R=ZDX|u=o1i{a1gxcjIN=Y<BY*-LwewS`qY!jiPv31wDKAnb{wI1waoz`;+W;s@M%CnE=|@Z}0i14xf7do3CEj|LQiP@#Y7!m+tKS@plLB-aL5sgTuGp17Lr<_nW;pKb+ltcJ|iGd+$8A|NfV=fB#^1<L>OnN3&Z`%Mnf&N8r*|9boUx=V!Oxn0@@S*_(F(w3zKv&+q->zYhNU^ZlRwW$(u8`(NIGi^Gq8HT&ZC67(4Y9XT<s4gx)V>(_fv{dE7myL+#^z5msJ02ROa^B?y<c^3z`eRJ=%PsI#Azq9|v*N8s7I{5n4*>B!H{5jzMEPo{jIEw@HCjE6*U#FMf-2e1FK#Z@%;d@W--~Q$7^H26){a|+YD+%C11mL-$$AW<M5yrv$PtR_=0u5&`y}tL^4cYJ<jRcwo<lGU|qUMCp{+BNueEr<Pdmrt+_{!n0Z|%SF^ugC}@4fgt82<3Bx5cFB3Buc5x^Vu|rSF`-@O@ys3-*QYK6&}VwI4lx(YRoH4zRPibou<XM}Kh9`OcG%eDC5l=jz4tPh7d2bLmP;1FEJ@g0q%<tF!H~8={hQ0)OPoH*3_k&zx}-1rFODx|1ZunM&@`rGPk|3%b)!r+UU0I?6V3-FT8#U&Gq!$s``Q9&>&&Ve!-%xPFj9K3jM_j)Lhr;8lU}V4(@;yJCyorKP1l8)yz+UWcK)8wsd*4+Q=A@jgpoyu@q~EYPXv`+Y6yp>}sskqb4cZ4Ch{v_SYqS`4=BgVll?4qPMP(5268)XoTT>`nvFA3$_eh<1N68Yjk%&JoaCS`Cw3OJiYz>fB8{-*11{4HCv@s{BjX*p1yZij%gXTgXc--889v%)l&!WmRWzRn<A$?W}YyEx%jsY9};Z`?rGe)!~W2-xOpNCdvRhOaf4t9R0S=O|o96s6WtnTdm!O)y(Dr77i?#dR1btE#c?j%FwES%5yx<r)2}^4AWdW(O@;DdF$hYPeH6qoT(SMNuqJ@gpWil1VZJfj$<S&7+7Sm%p4DVX!nDF%i<%KE*W693~Xcr3Q@4h3{$KN@kJV=%aA6PHUI+SZ_EY>_4b_2$nQI23$7o(^62Gj7q4bAU42HKYfmT8^z7+;v}DY@Bolo3r=wj1W11o(8XZDOj8g;%&&_W3#<XfMz&}glXexm~+soi8C>px`em+uhxVhkka3^_JNiq4GA&mr`3k!9Kv%#k233GBS(491Q-j}c6(SrgLEP%Y!=4Fnk%x=zbyV(`E9N^-j-)6Y+Isrl<FrhDJ1=|PbD=`Y}OWi_X5s(=-1<sZHY!M{0($EID<N{iF1K{xndKZVwx;&s=8KX1m2CdAM1pxuS&IftfisB9E)0?Ew4}GhoupuB9a{`hJIso;tU6wbYKEzHyg*>fVv6U{kGXn%VMFGoCG_Z}-v`nOW+l&@~faIyFFrHd7yGMb^sVLS~=-f@%eSlIMql+q5OlbC818B#!Qzma>Z-Z>n#Z+Eg$4O?F`OGwMh#7h3cIbvu$8WlUkDk9nJxb#O7%_-PK!Bf&6-!$+ahHM*V0U8>JU<zrwq(YpNjo&Rm*C|??c6UXHr*r`Rm9CQzd<_{_6XO&yYh&lDdB{}X7>+kYb)?)n)9uP%1`F5@4;u!%%1zA?#3V7{Q2xRpX#~kcnrGFF&y58rFrm=9L6@WW5tm0HgxI9Uuq59QR!dqI0T{utYk)voS3v#;t*`yMHzH8a>q%>>*md*m8ZAdw4*fHnY}O&kfp)?m9EJC^roP$i~<s-S@jQc06<gptYCi|rfttG&jB3{))&+6O;gk|bPb9dg$x{417HK=?d(?UFY8U#9qF@yz0+aqzmDdu>}2ZT7RuG}f+%5xcVQ8vQs6UDXaEIigc&T6px#lvKsh#Y6H&!Th<p$HZ==W$6#mHXK@1$CPXQ*<Q}7;AumBV#5FU|T0L=@dRvHv(k)u{=aD*Gyd9Yii1FU&GrFE2q1Vg|Qa@P#W^Tz>`iW&e!B(gc1pp<ifpc?Axn%J2*ZNikZBK`tLmtb^sy4Aa&D{|~c?zThEo2FfMW0k(TKAxsSQ1Uoa2QepKxz4m|Pd}FxNuCheItZ*Cz#asb%~W^gRHCLy8wIvNI}jrAnx@w-2%HjDHmGP{rkSN>gla2@S6)k@iSl-&jtF46q0~i9QU^X|QQ|DEqMDRS2pv%jYL&>RTn~i=3yx{$W7OMlvf79o!p>!p;~*JEY;LNuUU$R_69;Q_q(d_Z)zTo8$c`X|gaGQX;A4Tcas*odjrCl`!rD|cA%$IqF5_;FVO9ndp4lKQuTo|&2!v@W(X26H4R2+HL+tyoiG?%U2&1h~SI9>hoP!&8fGK)5w&o~9!-2lCy(2=;;51EGYvtVbu3qk$laFnCY@BKr=?BBf;M&5oGD#oM9?7HO7z&wyTd0jDNlJbXc*p@*G44jk{<;rNwTwQ9*7kxco;%t2uwRb5)aG7j&G*mX%7e3NzywU&ad3~|I0dW9BQc3|c?v!x$BJ*ty=W4moi01FYo=jl8=L||#a>Bcl=XnzC6?K&<U3p0xG1?|Pn5GFA!C;02uEydr<6j}$sKw@TOrL7Nwuh(Ioz|{|1LN&S$M@$X{~G{++aH3X&Do<>OGLM;}h?+Xuh0ry9Ese`bw-C5T)!A#i+oz+n@PkW!j*97Rw0~=ev^o)8}%BUS3BSnVc7;HqSQo3#)KMyaO=1fUIum_K$%-VBa#vm<=DNM!IspoLm$V5Cmhe(narS3{CxQ)PA_9Xzk>4hHV8{^mMu_9(ubiz2uc4D5NuitodWO2>8YZ?E@R;Ek7L^d@yrC_ot@I@ek}~53ohZ9TX7>vTG;fz)uYfE!H}1K(7t#mQKb2BPvp~!j+vYjH+4H*S*r~8dR7hVzA+w+P6Rg(aWRz9=~tRag!=D6>9ef$misjE=ZxH7&Tw`j7!UWe|!TsM%8CMwMW7uOHB-%aT|t`1hdi7TiSZ$29_`ZiVaMo)4r&~?zv-F+p}lRoH~;|Bb|}Or$~<g^JC%w9rB7gHxL3&p0s))JP8lsRaw`o)V}8i0VPb^S%}OC+)=OZwlElGROe|fXfZNcR;>k5*352^!lop#m7y~yCSUuj5X>D{txkR~$yk`h|C)KJ3H^L~LvhPRZH<~7V>d~djKH;vBy{9N+wy{&A~hVDJVIi%3x><mDru!Q5_iEp)cmc{yxml7yEK36qlQIn*6z>`#=iK)R9;-~s=FaOwmbMgcB0>@_`P0`TcP$!0<QEp;elO{-STSGRYq>y(5Ir9CneAGjXP)dj*mj$qW@ugeX47cUGveX#csIHjKVS&f}_H^3Fy#+Q=<=_3yL^*<&w@y7vp*M${IGI$yZw}ugWGzE51B86!ximo*v?vSxkW6xO?!|zs_F#?d<(uV7i(T)dhR_fc9O~c&FB2=N65ah)B3tCcygtL>~JjVX>ZQ|NJ9Oj+1~&7`bTYw_Pu_E5wCa0)5Nmt+Pwk*1m350RRzd5UQ+`H!T84cnt(CaJS@|_?T7zeMsp97EVSi#*pB3e=JC!fn05M&3Z7&&6lc25#d#~e5R%H05E#%+Sj#Z@D?b3YoAja{<&+Kq->n$*k?DNnZ5kc-Y?(T`_pSYqkj0*8wX!MxBte|yy{^8!=KJx`jx`bQF|Al(Hy_OppUmBy+{KsLC5c!^=Bu5Z+G>I&&t=rcE@Aj97EKn7<?HlO-{8XDxI_x_GrjeC$t|vcID(_Yg$(Nr01=5OJc9`cKL`_RUhQZ(M=3;KMoUZ40Mm&&>yfQtp>N_D8fyjd=1o!VXQn&T8PeX$;3G--o(hd(2f-ik-<XPw!@;g^+`HdKBreUaPAgG1M~z;-N@C+dt61Gk-gdQ!#;nmyZsm}+VXk~nyJ4m$y91R0teRFWHHf_Lp!U|FS)yE=ud{pWIU$$YO(SRnMAK*+tCx2VogD!cRTu)8*ETW#tZyW&zZV$p9d%Ha{{Y!dv_fO>BMf|pCno9hcx@l--#l@)3OmNae5eD1m-2H%b5Md4^Ski0U;?HV3d#^z=Yrl;w*{TtpI0U6!)ECDzL3xiwok%t+kt^yfCQJ6oX&orex<aDPW{9K4JpDakM3_Kt%Me+0s%{aw)e{fGKs;yOm4>=1dS42UYKd?~9$pekMnRc!%N;g+VB1h2i^b3~IK6Ikw%S*6`81;1HnnVY}sfpqPv639Qbx3(KgSX-#4CIjR$WE2zLF9f8%rt*^Wxy3(7b@OgO>lzuECjc9=a18(p`OdXZc7qS0P>l`l-9^8=PlmycSelK=oV8X6)M{VTV>7>jDkPqB7bLE5ymY4<H_f4a`kGaV7E*Hf=9cnBAe-*g0q6>rsxP7CHccq0Wgl&sbl@k8tj@d-2Tr50PfN|;^?#NbG*5psV>m*D>h=U^Zt>!I-2<*?Nr7K)$qvo$(d7{}<oonSsybc$P+Mry(_xf|+eEHe_y)S0BKK|RiH+DEK#}Joe-Ca=OROAU`RYjE|`zU>t%n%?aN?q+_=&qeP%Ujw*w%zyF!8jSkUc5<FsOHWDTqm|3Q~8>5E_?61d-&l^Ua+K^QPq|sdVDRv&D{fVPKQxHTV0pT9rxW(P|$d^SBufwXbX4w0om+yXv5AP@HMhT{<28ln<umKThp4`bXsg$#dpVuVK}~Bi1Y0N@G7Vtz-5`Eh(q|HarQJmJHelwSXG3Q5RO-c1JkL&foBCcBAET{N};r*8ZMQPiN;huwgI9&rf6z5OVHG3U_G&KuIMn%A5_|53!7)J%|!$6%G!?XEyfXrVnCEdrH6^aILay-O2cQVWxL2l>U0iukm!k#^S~C!f-0~@1=d7YOOti86ccTO++Js)s0Ap{pYO%9GS;MhW`(Lga_nh5K7X?zKj!HzZH${GeVeXeyQEL)3Va;>I$d2C93nwVpV^?|DodmKS@i&&9i4BH08mGwLX0}5;JVG<g54@qirS(+QXO~F!6^A|&ks;>VC%v?fDxWPy%E>DN<fH2f=JLTIxU(-w_BKMo}Z!u3+`CAv{e`dS21L9<{-BKE#Q!4Ssc8)xvDqC&2(CKSY9`ECMhm}Jbqw@(Uu`kAlUE@jc~;4>J76IUhYCTnfuwQ_^Zu*$-WEw=?E>}%KYAvX_Y#D{>+eP_&!{O7j@+D>?w~jHbE${K9vnqwMOYNf-JK>G%P=8gim6I`Bbc{0s~0*GKVbUh;!ndl^IfRyJ4R6ZtiS3L?KN?bRk#*ZnigFCBdExCXFW{dcnx~Xw}OgFs492b{i=}r$5rzB^m6XL`p9k?$K-Hq!RgwDtL@;5c~yqE%{xueV~3a3hbcBIWpp8)OBY?ZAw+Sc=Wo$K?M|R)j5Tdq)Ii?Vg*A-9HA)tnHd_e`JjxZ-jx+!JUy=`6VGD_rb{dQNfea)xey`BJk0xxynrqvBv?8?FXV#-K?C2cr$c9ZngNjUvHJsXinRM=#0xMHhGDp1xm>VWu6-SoFFm;J!7YZ<%=^e+RP2Lm_0U3K#E&B^;(o;&9{gKy1}b3}i<Alb+L6LON9@bMvQp69MwZHiyjzmW(hKBs6>t!Gg08as)b{>LLKfik>_~$sVUe{%082i!6!jX5QWHG4gqKk!9B75@%fn?aL}AkIoFszMu`Ny*YkHGm?iJ8-EaVp~DbmR$Qs#=iD;c7$rqn%nB8rfi3wA+pH6|;*>a@^;;OU&k7R%V8)tx(9iJ|FGQI$Z&f-?*K$!Y)&7L+<Fv^lp$iHj9RRkLOkEk+(IZfML$FLY@wscNGVex6qIhPbH{Hc=>iIx-vk>#$vep*DSwX|4zU95{S-@lXPmkf+he$8_XWmR9w}Ip8F8$H@@vdk^<Dn5RiyKZMt~ozB7FA=U^T!D`5{X12$1^ke3w(r{yFL`J?F1K@lSW6DG02olCgSRP<NV11pgarVnUnYiTqt3q2Dkmf*^fW1%{t5BnsqbFlo{HQho(;KE@k2c`i;PXz}7}mA;X*ws`pxbgsij}nsW5J}+hgwGO$HoMKa@OV3SJeqCXmY=kFU9bDI=LcX&h=I>(9W~#D2(!)n6g0DC>$Hifhf-is*eI%JOv3FKjiJq6Z*e~x@lA5gc&kzqEwd@0()3HcO2t^^q)fXC!-PP|DBE2CNC+s=vYUNuRL!=CEQscS?oHjqB66ji8FVbMzSYkv;c<GXma|65=GjyihO(l`6JC8AG_T=k=z28ru40L?!xRUw0?ap#a#Zc7gAQXpuNWuEpg-?<S)T%QQ_6rZLUIKsAMwWPtp-)O@X1?8B>>G7gZk~*Ei}1#VDW@CH9~nje(yeHu_dir>GLJK66J3o}%aWR_u$jCT3HUK`=>%M*TGonoL8_knQke3z1PhScG8`Qu{_W8*#F;>Z2JVGHn%ffu&sopNcNc1za~0dzzD|p+)Jhdu4bIw?&6<rMETTUBV1awiJ!$)*6fcN=s=^!oUwV3=vINYvX#jzSUU#Q!x0IooYj@uW7$JZy(wi-{f1ornj_#D^<)<Nri{udn%4tB>&L&2Lpy?%*hIjs%MMwQw%@<>frMaX1Cry{PnYXW5896pT&sPf!7DL?=v<=mj(>JhPqP&PYCG1KgfY1jR<*2Jmdv=<Z|__{opbtg)7ixu>kJ8;lB_BCb6|2`az$?A-Yp24+$2p5U2rul!PMNTrULEYQ?!29xF#^M%_yI0!x_-)g?(Db1a8XXi183Ol~z|aOv0<j}ZW&`n06F+0Elt6tC)(hovz%V9Es)v1loC2E-)1)|r}#jjfv3d37|OIF-_r%aJJ1q%3o(kxdqPRATe<2O(A#W5`%k*3`smz*U87QysD_5G3Y6$vGWGeG&GnmVsvn^(x$U3HcTIDEp!c&m7RRoOfdWB^LhuANs@sHgaPhcyp;9kI?dfgaly~(8O4T%E~ZG;$uc=RV1uD<_AWpLg}QDrK(P@nGN}+LRlL@J+W#wXj&N0KS^zr?~!uS!71idcnOz_%E7g_Phccx87|D@RB1iM2UB=3g$HK+*c_^Xda-_KrShRFN;Av#%sE_%Wk0?f$b)T^2M|S{3&)^K5eFGVaC!{Li(^z`HF@@lPAt~n@$&3)fc(DZl$GfH|4c~iLHgx-JIP;x+d~^j*&)Nw`R8(UVLvglr;B<-Zc9}gnIW-yDF>%4yGLFbHfy=D;t+NTD(6&6o>AUxjK{g>=)HNVaPd7C@J2v5y&)yUcZe3dyjbs>KfjoLu!w(E2#oFiAAZljEwumc?StF54&Hy|;Q3E@d;E|OeUWJIclTx=|4cU*E+-VF@u&EAQwRri)c)R&CM`9SHX|!4@nK><yxO{7EKgUkRDO9$Nge(<rrDjp%)WTFkw$ekP-8g6oMlYME8DzcUmLctsB^eHZj3;Sy(djP`*Z+x!WviVxGZj0`K^g1Zl})NACDc+NM$PQ8<$0C7ju?Ij>a4?;Zm6Z_hnK1u=GuUP`gI7I#nxL@P%t9mCH5I+}iWX*#|QkiL&u!UJBkw6cLD~Po0(XMRcgXV*h0-L!4y-0Tjm{9O?>aukem^Tk}@tds&V95rJdzBWngK1oejD{?)M@9yWgM#e=sCm(&+1fYtT5J*WCo0*^@JDms%8cBbAC>pS%$cvtzhRJ5Ph@T<3CO{<4J`Aer*^Xp+uY%&r%B_+rvx=EpF&KK(=o~6)v?K|!>MZR64r&Ih{oK%%0Y+hJh24|hhR7CbioMW2{AfGLC;sqdOVrA7JmB-unaS(`VNGknhK}qHD_1(GXO2TDI#3AofkgM_i9GG<vMLwxSw5TqN%B08}0nd5O(oQQ({81iOHe=~Fa;TUFrHe%g`wx3(N6lYl00jje)*7rc%r<tZ+BiPZk1g*+UzW64RE$QBH{SOPHEyCL&C06KmX6HD_PsgX5z&>0+K9nJXHS!*fLp``Zq@ZaxnoT;Fq<14>0zsRpd%Q7&faD$K+*8U4FqXcq5d9jSC;%foz48ikB2|~&x6nJ%<jE5yK|3!__6Z6$GP8<Y>0uyX_6mwCvwoi!?L(bi&mb<a&{PSo|RNt^5atBA14IN{2a##e&FyQH*OjIPqdp8z$@lLhB5oNr<UuMc3V4rlv(5-tU_*eu!7pa(}q}B6BRyKl|QS06GKoiOWeuxCHN(?yxYlQE!oNJ3`(AM61%4C4P~-QpI566RAOxTGCxw29dzO3Q>{(q8&*q4hN{2Mrq!UerJaXW^Ls$~h9LG4g~}9At(7UyZa#zCqL=P~&`^Q$wpS{&{vR{S*f{'},
 'road_corridor': {'sha256': 'f42e3cfd5b8d6053a6452afc043278512189b559913c884cdbbcd0dd1bedf7b3',
                   'bytes': 17189,
                   'b85': 'c-qBV-H#*3aewDu(KtVFW_LK+-P1WXBMNZ>JIISI7)Cxo5X6wvB<FTGL(dFvmn-ap5h0KP+mRC~Mq<N>4H)rD?7%VNAW)!xCcAff%D<4R>hGB*mwOg$;<O^CyQ`|JtLx*}Jx`uI`Qvh1q&#JdvbsvDlvQPdKk~f1%8Di1*7&zsakeeytE5;$*CJ`M7o2U9X2rhr&Y#7--k)x>l(QGSYO*=6SyFLUmy3pND!$+qFW_O!z6|X<#@AWX0Fjz6*Su)hyj;VGMOCg@(t{tHoHtyt_~sUd{2}|xzk+U6m8E6H<|{tGtXaW<8k<)<f!+zbO7cq-R8oz5-&kceTbJoJ=PWIOLs2#?D?l2S6!4|tV%B*HGzE9oT$O_~gYk7%7D-O3h<i_-Jn2z~>2$GewiTaFS+?GkRRffODo9!ubx-t3lO~zxNnHa+*-1a@$p>rDo9rHgggn<>-^qa@OykQ{RaOyurvy18_Vsex@LsR?N19RqgKu~-`bM?oVUHfMuh5dcQ|8%xH|#O^B(Pv*HLW+v9B2-4u1Uj2><jT3wZqgelNC==)Aac>*{0NOnx4v*is#8SPp91;Uz9yk-jv(Q)8XPxw}(f7^9v=xnm1K8?@E6z+h@uAvZ>(Dv?+65!DP9`Nl$T}<kQ(UUGip#s}5~%o_+E2VkV$5q%xhaw#8+w+ArKj5TIzw*XyNx!KT-{V1r8(4y9(mItnJ9#n%xNUv{DDpYvkbtVDb54yw270PP}9v$_e8auP<wVHSqCH(3&~8A!-(vW=u58PwzR;Uv^d>IAot?jrT4xz*bo=P}m8*W<HEIHA@f*a$MDTAB=yN=RnZKgg<^=ipY5(K@*f&Lgt<c+RstkmKIal9@5t0;vW<$dH{%B5%`@8O}73B&#|5^CaJL+AKkTzO7#Hsj?bU!Zgpo+4KzqoZuzSu0ltKatRKn>^yYoFTk>JU&9P8L00PK3P3`}$z}tNBnXoD8mQ<|CoGu7*8_M8tC+;QPTLOJcA=cRv;jd<t|RCaoLj@Ei;9EGYXV#tDI|ELuT?4d)s)S$G=mO}a9E@Rrj|pjXAUkh-7%q&lw2wHs$=wPNDX}Si>%z%Xce?f;3X#@B`$?rP$ZOr9iZxXv0cM<YdBb_(OFeq9T=}@JdfBnT^h5N-e(;8aUccT4DBIfqY-<Kf6f><O3v0<5h#hr&nFRb9zP$PPq6oM*+)tdIi!#H7{Y#np=$!jc_M9G2rdnsHt_}~YZ<qtW*BI%yXFc^<?3&$atY>8Pqmi|!~()@)FIVa!&f;(+`7TVRU;XMzk0iYAs7!6jG|eC4>1mIYp{f-WUyD(Y{^U9H9Kt(=<$G$HX~S9(6jQ(Z6~3{u_Sw(s}S2Bdbp-QVwepEEF;;^$QHUQqD4L($*zutM-((D%Nen+mIZfb1s;2cvyz%N(#-R+M)it8jzzm-l~uOP3Zj9+yYoqbK5WDhlb!)a3OC^NQa%#|i^$l|@k<a0XL6FFn+b&XI7xRQ_$KL5PsIPiwn{F5pcp-9V0d_Pa-m7!lN~<kxw}A`&`3?_lr<-oKDAzwBF$1vz<@=10iJGF2Ws>{X+6XR*48gA5EUu*mg*nrZ|($Ou+l$_rE!=AYSwS5=3!^In|0IOdw#M?B$oy_%8FAIlU>IbleTQWiFGB>c6Vy6t{OU!e8Qg37a@1iZu<;HQ2Gm%YsvUm>(FiO&JCr8DiOe(Qie%n(M}sg=h#icWVC{;*fX-j$v;?#|8Bn&0c@(aEaW5oVK?eIaz+mC&~@*9lMoUqj|*)R(~h)7N%Rv2FNn2Lw+Q;E!QOs`^sUOvL^fUrODvTjF9V+px&5+C^48MmrGvRbKS-2?52g{|h{yDw_FA&ynPyPWfKO3nl@n!&XDpj)%QI*7l<K{aE}S5}&yrb|XASwnppRzAuXx&rN$YLYcf2>1R)SJ%1C>RS6<gP3VwK3RmAvAYE3T5oNt8Pog3L-WXnJG7NOm<bNAT*n7d_%F$nW{!4?K>fJr63c^0HVe_1Tkxg!RmgWDVcczFb&yenagUw`L2;cUp|dW1hR2D%t=raMD`Av76Sa2txzj5ZyW*ZqS`e-QAGymzs-j2I{epOn0)qI@kzP{k!hoiJ;oo(Nd~dPi3!SIgb>VYe$r8B?@NP^6Sp&u~R*E(Bq~R7L_AYJdKXK{lpejJ)Qj~I%62n#%Rn9_jU-z;M$ZsSL)OAN%WNj)Mc7nAyA8=4VBs^j$*(XB5eT#RCAqzjUn{BMzyC&XoaZ<#O%~f?Hph4^JkGEsQe??Jz__J^b62V#cLsj$P)_WVuPK^J#e+kaxR+S5u^fGbHK=uj1azzRojFO=r@g@;7+Hy-lVA1ZzSo0r(N=fDk5w!`qD)ly2kif#Nvx*cA|^*Cg!<@6Qo*7j>*704^I>+Jy<Z6Zi0H9!cH#ubrS^H;9L}8W|!+8Nc!|#cnYLH0h1#^LeXYfzMPj@L-8~0d7313XCdnn_7!e`MvlZlSRoEqa|uO>wG`;#l%h_s=Jl!-?qmprc~1E?Cdvq)jh>z1Ct@cN!*I<<B10FBk$&g^iBBj3&==5)4Sof3LP&#Z7u1aSr4mT;MD)l4%2ax>$dhHQ47Dap#5CuK_}#OT#)wHnrWvLjOdr9_3@Ji-vDpeYOhnGqKN;3nJX@}s={$iH2ztGHOHT<4ohlQfSaR_NJDT9?<7Rw=y@}w-(+eTc3if4L?IJXwS(c`}h%D$!!Zesv;KIU+p1JtfqQc<ED^>kS0B@lz#(}_WZ|Hje7JMTtYD^tX{J!M4tP((*D*(+#(kv#}S%A8Z!dymn%NRybxegW?iv}fquPvB}L3;(kfYON*xIu5I_!AqY!#hHmM|rZIr3t$kQn_@avVV-Jlb+?)gsYQ2O&DJ|ICu0+Gnu2)&2wNF`Y<vFQx5vNH|BG%{`I}P_y6whH@~|3+kd$K&-eQ_d{-2FjyQd*=uMi2uQwgzq#XxVLOv6cM%YMwT?nY^L_g5LZ|H$&#At)+8ai$0`B-`a1s0&Qq1J+_+s_TniIct}Vjr=qEX9Ifcfw_-`L_~a5!BFs_~M)4;f(qm`->GXG;~)k7IXk34pR{IGcrS&Bvy*x9O$&js=8@W12XefGRwI><RLQ#nIQoL*~<lZDLfFt6O@EuDg+O}F3~Z^?Q|IxfPkE<R!`C_CLGHM5rmzzRd^7YG=l==h%TvY(dnz5+0YB}c{6s}TggsLex%&~*d{&cW{IklL(A6A{X`9xfpTaQIX$0r>6C@46_R2Xm`EciIO8XN-y#zPv^42@)nr1y5~Mq|Nk?*Q?D1b+zv^uE;a)T~Tj(gq-X68$I)-N%`LNN7V#9;*tk-*qlN@wipMLA_U;XBncOQRr_sfs(KlsthU;p&epT76<7e4@J_VUA@-~Idl{NKkvy#Lit?tk>{SD$=%_ro85`qw|dd;f1>$jkr!q#y3(EEr&rK%7~57juZz&RPa);*@`>HI|^Ux8q*c<jhk(+b*YQy|5$Ez4jqTSn~dm39l$OFlqqzAv`C4^hlt)FUw3kgX|AgrlOuUB|LMC=CS0}T&JP}19Bz{>Th64r8ZG~M@zSRIVls;IlAxDRJ+4QwSj!Aj?Cc{svM881HG}uyd!A{;X!KJ=s~FN36a0Z1+F=s4$z@)mu<5p6b{7<rJSL{K@L)1^CrRbVP!MTwnn27=jc;)u8wviA1UFX7qPiLyc;b@45OKghaDcu7u|sdAUik~_LRf<ZFUhqJFzI9vWv6fgXXD}6K^*u0ZgQy5!fk&n;NyDlY<uaH?n^}`aFqwP{NUs{sHUCjYoO>6d!>IKO{!YAJ5(c?~VxcwozJo@_3ydTA|-c<4(9!)j}YBhT5TOru#{uRc%_AIapHT^Wo9{1bBfsRRDt`<50ms&mP<{Fa%O=NYZ2!5BfzpB`t~V{{r3K6n@uG$=Z)Z!GcdroSi}RyM%?5efr_Uwm^~lH54uCeWhp1i>V2Do!0?H-9ETR<^sV{3-n02`|<=tTtfiqz{@0A4lT71c%OvQ<SJg5`3q3KqMa3s5);Ud7+xyL9Op<SoG{`JOpEl!O5O=2_3eQ1;A$H>z9Sui+NfB11R>`nhbYKQ%yZ|*uRy|f6qfaFAM+du;8^KSQ<`UPj@%dE&s1W)xL^Vea%zRa`Q#Yy<s@pXuN^aK1)j@NvM)%(1IZ*d$@<Qu?Z>fHOApl4SEyM|P`$T%6Mxln{~oYWfmT_u&tz$K2l8#GO)q`*o@V8jraSKN7`MOOSAl}VHZ_TxkzKa%vbQuM(Yiv`t@`p*)~ph{?n*`}vQne`wGlV}<gYrr0n3>J2`N+O&`Vrd(m<WzN(odO$#Y=PPP1<{5L#623QD&Hw>PPKbI!YZavG4>ZUVn;=<+^J(lpTX*<-sxcjmZ$zfQg5Q=Usqc5G>U==Z8EQ`lRN_yE+sD)#s+={5RBo#moY4hxI585kz4Vrrb2R?|^F0<W8`dDkt<2vBoa<Q1={6<;S=0Tzxs8a)a^{d>HK7k1$kUq|g)p6Xdd(<xSL3yo5l^8V9*eDBpK-@pICzuf=FM=wA8=ll0RxckL-?mznV|Nhs9cOQKJ?%Use`A_fN|J%=A{p35ZKKb$e_kMPK%}+ALI^h~IZ+R|yA?}u4t6+c?oT$=-$@u6>)heLLzQ57#h`P=2=+T0i1o<|?GzpI*Fs1V$tOI+mQTMxObJvhsp%Lv3F*^#8gnGYQiz8_Pgo~&tATBzHagvbR7G8Vn!LMna`Z*qlwL*@HG%9?Fz=~>DF5X7>L0gC*<0W!kkdMty{~Y@gtG9D33m4m5pjCK#2r{-T&+)V?tFuPIv2#w4DP46JN{cN@e*!ra-h^*oYOnYvhpkS=6v>P1^#*(=CS`QZo@2_*auC(fgK}`0!S0x<xCC-%@@D4By+0v)=p#X+G;svE8lO$}dhCg%EvZ3^rCQ{+G(a)xh=^h8mZaziPoZb*BI~lvV7wgxhOHY?8Cb0{2Kpi6ruZSkrueRGLP_TE>RjsbDb3=BC#=T>JFh=PH!bpUSi%z-c#OzH(BuiVPrAcA7?Lw{7e~^ZLO+Ov?bkaCmN7g!w`4@$fyK?7_{)Q2G^zO6ky`l7K;z#XP<#|ZtBtlFwf(Jaut#O_mS46V-ywYU7`n$JuzOKgFVbo`HXp|7HPlDr?A^rUVHmdHSC8E~)?C8<2&{8B<WSQGMAo815I4){JtUG~9R;rzerM+3!|=R?$5Xo4=e$`G(8c2fE6OCk2_t$>fo>w>pBw$IiM|zNJVIo7Ao<FPPtTtPQ+uxF`yQsswOf|dJt)TO5^jRvM25$skm_?1tj13#B7V_T1YdwbxkFFo*ECcX1aghXotvi+rIq82pHU?$Rs5D*V0hd+45?_^uMN*QRDL^bW>1YHOAg0%x|od<>mB6Aa*Vy>Q4yj0LvW0A0I74oSFFW@jPzK71w^GNb8D5G4IBeGJwKkNl;>$G<$0PKX(=VQI|y@mkocZ6A;gU2%6kevx&Vr5^cVS<N<%`5FN~5J*m_$xl=_2pU}8_NgU&$vOFjUTfap12?V)7y^_YE~l6Y1Y`3`S3<ob2gGT$xBVlcxOT9azGhm?#1WkE^26SBm$0=ghUn<a)K1y#I<Qn^?R=pER33Ei^--$uZ695RCngh>!wk{2y%V1zH-#9mT(OAmY;6c*x*v$K7Wh*jaUsra|o=U~Nn0LfD~xy6MYOAX`8NvvNCG%216XIg>@`0>l|NWa;7@y?kr-C?BH{;Y^QIds_oZOBaEFl6rs7~7Y{F^+2WV<0i)!$XqTSI5b5JaDJMQ-A6=oUFCAmqQ6=t;*|1E2}a(RfH8rdw4C|HmbhvhqAExgNUZ4tPU2;%<;rQu6$Q5;UVDAD&&R-Dmf3nXO~&WVTt>y%tHq^9Vo(aGyCIh*H|BQmiW5ohrg#YL;pG9BT@t_p3`l4Jh*9}8Mflq@#lV@ksOaY15~)UatW?63O6ZH$F`A$T`J;E-vhf#9Zr}uN;e5kij2S!dfj*9sDa*mu--nvOTYwih^y9ATJ@Y;9iBsp*4d<Zm8R3Pb;;F%<0CKZeioTKtnz}dsQ;Wx+zzN~?rn#?{|7(A8%O'}}
ACCEPTANCE_GATES = {'short_A_to_B': {'min_length_3d': 40,
                  'max_length_3d': 45,
                  'max_expanded': 500000,
                  'max_reversed_axis_yards': 1,
                  'start_surface_z': 28.680936813354492,
                  'goal_surface_z': 28.71369171142578,
                  'max_road_deviation_yards': 10},
 'short_B_to_A': {'min_length_3d': 40,
                  'max_length_3d': 45,
                  'max_expanded': 500000,
                  'max_reversed_axis_yards': 1,
                  'start_surface_z': 28.71369171142578,
                  'goal_surface_z': 28.680936813354492,
                  'max_road_deviation_yards': 10},
 'known_long_route': {'min_length_3d': 3450,
                      'max_length_3d': 3650,
                      'max_expanded': 500000,
                      'start_surface_z': 56.24121856689453,
                      'goal_surface_z': 27.7370548248291,
                      'max_road_deviation_yards': 10},
 'known_long_reverse': {'min_length_3d': 3450,
                        'max_length_3d': 3650,
                        'max_expanded': 500000,
                        'start_surface_z': 27.7370548248291,
                        'goal_surface_z': 56.24121856689453,
                        'max_road_deviation_yards': 10}}


def say(message):
    print(message, flush=True)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode('utf-8')


def beneath(base, path):
    base = Path(base).resolve()
    path = Path(path)
    result = path.resolve()
    if not result.is_relative_to(base):
        raise ValueError('Path escapes the permitted directory: ' + str(path))
    current = base
    for part in path.relative_to(base).parts:
        current /= part
        try:
            details = current.lstat()
        except FileNotFoundError:
            break
        if stat.S_ISLNK(details.st_mode) or getattr(details, 'st_file_attributes', 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT:
            raise ValueError('Linked/reparse path is not accepted: ' + str(current))
    return result


def asset(base, relative):
    if not isinstance(relative, str) or '\\' in relative or ':' in relative:
        raise ValueError('Invalid relative navigation path')
    name = PurePosixPath(relative)
    if name.is_absolute() or not name.parts or any(p in ('.', '..') for p in name.parts):
        raise ValueError('Unsafe relative navigation path')
    return beneath(base, Path(base).joinpath(*name.parts))


def read(path):
    if not path.is_file() or path.stat().st_size > FILE_LIMIT:
        raise ValueError('Missing or oversized input: ' + str(path))
    with path.open('rb') as stream:
        data = stream.read(FILE_LIMIT + 1)
    if len(data) > FILE_LIMIT:
        raise ValueError('Input grew beyond the read limit: ' + str(path))
    return data


def remember(snapshots, path, data):
    value = {'bytes': len(data), 'sha256': sha(data)}
    previous = snapshots.get(path, ANY)
    if previous is not ANY and previous != value:
        raise RuntimeError('Input changed during validation: ' + str(path))
    snapshots[path] = value


def recheck(snapshots):
    for path, value in snapshots.items():
        if value is None:
            if path.exists() or path.is_symlink():
                raise RuntimeError('A previously absent file appeared: ' + str(path))
        else:
            data = read(path)
            if len(data) != value['bytes'] or sha(data) != value['sha256']:
                raise RuntimeError('Input changed during validation: ' + str(path))


def atomic_replace(path, data, expected=ANY):
    handle, name = tempfile.mkstemp(prefix='.route-quality-', suffix='.tmp', dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(handle, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        # An optimistic comparison immediately before atomic replacement, not
        # an operating-system compare-and-swap operation.
        if expected is None:
            if path.exists() or path.is_symlink():
                raise RuntimeError('Destination appeared before replacement: ' + str(path))
        elif expected is not ANY and read(path) != expected:
            raise RuntimeError('Destination changed before replacement: ' + str(path))
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def durable_new(path, data):
    with path.open('xb') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())


def payload_bytes():
    if not PAYLOAD_READY or ACCEPTANCE_GATES is None:
        raise ValueError('This installer is an unfinished scaffold; no code will be installed.')
    if set(PAYLOAD) != {'navmesh', 'road_corridor'}:
        raise ValueError('Unexpected candidate module set')
    result = {}
    for name, item in PAYLOAD.items():
        data = zlib.decompress(base64.b85decode(item['b85']))
        if len(data) != item['bytes'] or len(data) > FILE_LIMIT or sha(data) != item['sha256']:
            raise ValueError('Embedded module integrity check failed: ' + name)
        compile(data, name + '.py', 'exec')
        result[name] = data
    return result


def checked_asset(nav, entry, snapshots):
    path = asset(nav, entry['path'])
    data = read(path)
    if len(data) != entry['bytes'] or sha(data) != entry['sha256']:
        raise ValueError('Navigation integrity check failed: ' + str(path))
    remember(snapshots, path, data)
    return path, data


def preflight(root, payload, snapshots):
    sources = {}
    for name, expected in SOURCE_HASHES.items():
        path = beneath(root, root / 'adrive' / (name + '.py'))
        data = read(path)
        if sha(data) != expected:
            raise ValueError('Captured dependency source changed: ' + str(path))
        remember(snapshots, path, data)
        sources[name] = data
    nav_source = beneath(root, root / 'adrive/navmesh.py')
    before = read(nav_source)
    if sha(before) not in (OLD_NAV_SHA, sha(payload['navmesh'])):
        raise ValueError('navmesh.py contains an unknown change; it will not be overwritten.')
    remember(snapshots, nav_source, before)
    helper = beneath(root, root / 'adrive/road_corridor.py')
    if helper.exists():
        helper_before = read(helper)
        if helper_before != payload['road_corridor']:
            raise ValueError('road_corridor.py already exists with other content; it will not be overwritten.')
        remember(snapshots, helper, helper_before)
    else:
        helper_before = None
        snapshots[helper] = None
    bundle_path = beneath(root, root / 'data/bundle.json')
    bundle_data = read(bundle_path)
    if sha(bundle_data) != BUNDLE_SHA:
        raise ValueError('Road bundle differs from the captured case.')
    remember(snapshots, bundle_path, bundle_data)
    nav = beneath(root, root / 'data/navigation')
    manifest = beneath(nav, nav / 'manifest.json')
    raw_bytes = read(manifest)
    raw = json.loads(raw_bytes.decode('utf-8-sig'))
    remember(snapshots, manifest, raw_bytes)
    if (raw.get('schema'), raw.get('kind'), raw.get('mmap_version')) != (1, 'adrive-ground-navmesh', 16):
        raise ValueError('Unsupported navigation manifest')
    for category, expected in [('client', CLIENT), ('settings', SETTINGS)]:
        if any(raw.get(category, {}).get(key) != value for key, value in expected.items()):
            raise ValueError('Captured client/profile no longer matches: ' + category)
    tiles = {(t['c'], t['row'], t['col']): t for t in raw['tiles']}
    if len(tiles) != len(raw['tiles']):
        raise ValueError('Duplicate tile coordinates')
    if sha(json.dumps(raw['tiles'], sort_keys=True).encode())[:16] != raw['identity']:
        raise ValueError('Navigation identity does not match its indexed tiles')
    if tiles.get((0, 50, 34), {}).get('sha256') != BRIDGE_TILE_SHA:
        raise ValueError('The verified bridge tile repair is not active.')
    maps = {m['c']: m for m in raw['maps']}
    if len(maps) != len(raw['maps']) or set(maps) != {0, 1} or maps[0]['sha256'] != MAP0_SHA:
        raise ValueError('Navigation map headers differ from the verified case')
    library = raw['library']
    if library['sha256'] != DLL_SHA or library['bytes'] != 169472:
        raise ValueError('Current native DLL differs from the captured version')
    entries = [*raw['maps'], *raw['tiles'], library]
    for entry in entries:
        asset(nav, entry['path'])
    dll_path, _ = checked_asset(nav, library, snapshots)
    checked_asset(nav, maps[0], snapshots)
    checked_asset(nav, tiles[(0, 50, 34)], snapshots)
    return dict(sources=sources, nav_source=nav_source, before=before, helper=helper,
                helper_before=helper_before, bundle_path=bundle_path, nav=nav,
                manifest=manifest, manifest_bytes=raw_bytes, raw=raw, dll_path=dll_path,
                allowed_entries={(e['path'], e['bytes'], e['sha256']) for e in entries})


@contextmanager
def candidate_modules(payload, inputs):
    name = '_route_quality_probe_' + uuid.uuid4().hex
    package = types.ModuleType(name)
    package.__path__ = []
    package.__package__ = name
    sys.modules[name] = package
    modules = {}
    try:
        for short in ('models', 'data', 'road_corridor', 'navmesh', 'planner'):
            content = payload[short] if short in payload else inputs['sources'][short]
            module_name = name + '.' + short
            module = types.ModuleType(module_name)
            module.__package__ = name
            module.__file__ = str(inputs['nav_source'].parent / (short + '.py'))
            sys.modules[module_name] = module
            setattr(package, short, module)
            exec(compile(content, module.__file__, 'exec'), module.__dict__)
            modules[short] = module
        yield modules
    finally:
        for module_name in list(sys.modules):
            if module_name == name or module_name.startswith(name + '.'):
                del sys.modules[module_name]


BRIDGE_WINDOWS = (
    {'name': 'earlier_small_bridge',
     'north_bounds': (-9628.0, -9600.0), 'west_bounds': (-1140.0, -1025.0),
     'forward_from': (-9619.308, -1030.368), 'forward_to': (-9613.542, -1134.375)},
    {'name': 'repaired_bridge',
     'north_bounds': (-10090.0, -10050.0), 'west_bounds': (-1510.0, -1455.0),
     'forward_from': (-10065.478503779887, -1507.212281364077),
     'forward_to': (-10079.181030588508, -1468.9011349807895)},
)


def clip_segment(a, b, bounds):
    lower, upper = 0.0, 1.0
    for k, (lo, hi) in enumerate(bounds):
        delta = b[k] - a[k]
        if abs(delta) < 1e-14:
            if not lo <= a[k] <= hi:
                return None
        else:
            p, q = (lo-a[k])/delta, (hi-a[k])/delta
            if p > q:
                p, q = q, p
            lower, upper = max(lower, p), min(upper, q)
            if lower > upper:
                return None
    return ([a[k]+lower*(b[k]-a[k]) for k in (0, 1)],
            [a[k]+upper*(b[k]-a[k]) for k in (0, 1)])


def bridge_window_checks(points, *, reverse=False):
    """Measure the two fixed regression windows; never edit route geometry."""
    results = []
    for window in BRIDGE_WINDOWS:
        bounds = (window['north_bounds'], window['west_bounds'])
        indices = [i for i, p in enumerate(points)
                   if all(lo <= p[k] <= hi for k, (lo, hi) in enumerate(bounds))]
        vector = [b-a for a, b in zip(window['forward_from'], window['forward_to'])]
        norm = math.hypot(*vector)
        axis = [v/norm*(-1 if reverse else 1) for v in vector]
        backward = 0.0
        for a, b in zip(points, points[1:]):
            clipped = clip_segment(a, b, bounds)
            if clipped is not None:
                u, v = clipped
                progress = sum((v[k]-u[k])*axis[k] for k in (0, 1))
                backward += max(0.0, -progress)
        progress = (sum((points[indices[-1]][k]-points[indices[0]][k])*axis[k] for k in (0, 1))
                    if len(indices) >= 2 else 0.0)
        results.append(dict(window=window['name'], reverse=reverse, actual_points_inside=len(indices),
                            net_forward_progress_yards=progress, backward_progress_yards=backward,
                            backward_tolerance_yards=1.0,
                            passed=len(indices) >= 2 and progress > 1.0 and backward <= 1.0))
    return results


def road_reference(start, goal, guide):
    """Original soft-road reference, without coarse terrain cell centres."""
    points = [[start.x, start.y]]
    for segment in guide.segments:
        if segment.reliability not in ('reviewed', 'candidate'):
            continue
        for p in segment.points:
            point = [p.x, p.y]
            if math.dist(points[-1], point) > 1e-5:
                points.append(point)
    last = [goal.x, goal.y]
    if math.dist(points[-1], last) > 1e-5 or len(points) == 1:
        points.append(last)
    else:
        points[-1] = last
    return points


def curve_samples(points):
    yield points[0][:2]
    for a, b in zip(points, points[1:]):
        distance = math.hypot(b[0]-a[0], b[1]-a[1])
        count = max(1, math.ceil(distance))
        for k in range(1, count+1):
            fraction = k/count
            yield (a[0]+fraction*(b[0]-a[0]), a[1]+fraction*(b[1]-a[1]))


def sampled_curve_distance(points, polyline):
    spans = [(a[0], a[1], b[0]-a[0], b[1]-a[1], (b[0]-a[0])**2+(b[1]-a[1])**2)
             for a, b in zip(polyline, polyline[1:])]
    maximum, sample_count = 0.0, 0
    for x, y in curve_samples(points):
        nearest = math.inf
        for ax, ay, dx, dy, denominator in spans:
            fraction = max(0.0, min(1.0, ((x-ax)*dx+(y-ay)*dy)/denominator)) if denominator else 0.0
            squared = (x-ax-fraction*dx)**2 + (y-ay-fraction*dy)**2
            nearest = min(nearest, squared)
        maximum = max(maximum, nearest)
        sample_count += 1
    return math.sqrt(maximum), sample_count


def route_record(label, route, guide, start, goal, identity):
    points = route.points
    values = [[p.x, p.y, p.z] for p in points]
    if len(values) < 2 or not all(v is not None and math.isfinite(v) for p in values for v in p):
        raise ValueError('Candidate returned an incomplete or non-finite path: ' + label)
    if not route.segments or any(p.c != start.c for p in points):
        raise ValueError('Candidate returned no verified segments or changed continent: ' + label)
    prefix = 'nav:%s:%d:' % (identity, start.c)
    if any(s.reliability != 'navmesh' or not s.edge_id.startswith(prefix) for s in route.segments):
        raise ValueError('Candidate returned a segment outside the current native navigation context: ' + label)
    for a, b in zip(route.segments, route.segments[1:]):
        p, q = a.points[-1], b.points[0]
        if math.dist((p.x, p.y, p.z), (q.x, q.y, q.z)) > .05:
            raise ValueError('Candidate route contains a discontinuous segment join: ' + label)
    for actual, requested in ((points[0], start), (points[-1], goal)):
        if abs(actual.x-requested.x) > 1e-5 or abs(actual.y-requested.y) > 1e-5:
            raise ValueError('Candidate relocated a requested XY endpoint: ' + label)
    length = sum(math.dist(a, b) for a, b in zip(values, values[1:]))
    dx, dy = goal.x-start.x, goal.y-start.y
    direct = math.hypot(dx, dy)
    progress = [((p.x-start.x)*dx + (p.y-start.y)*dy)/direct for p in points]
    reversed_axis = sum(max(0.0, a-b) for a, b in zip(progress, progress[1:]))
    reference = road_reference(start, goal, guide)
    path_to_road, path_samples = sampled_curve_distance(values, reference)
    road_to_path, road_samples = sampled_curve_distance(reference, values)
    record = dict(name=label, start=[start.x, start.y, start.z], goal=[goal.x, goal.y, goal.z],
                  points=values, length_xy=route.distance, length_3d=length,
                  expanded=route.expanded, reversed_endpoint_axis_yards=reversed_axis,
                  coarse_guide_segments=len(guide.segments), coarse_guide_expanded=guide.expanded,
                  guide_present=True, blocked_edges=[], road_reference=reference,
                  road_corridor_deviation=dict(route_to_reference_max_yards=path_to_road,
                                               reference_to_route_max_yards=road_to_path,
                                               sample_spacing_at_most_yards=1.0,
                                               route_sample_count=path_samples, reference_sample_count=road_samples))
    if label in ('known_long_route', 'known_long_reverse'):
        record['bridge_windows'] = bridge_window_checks(values, reverse=label == 'known_long_reverse')
    return record


def check_gate(record):
    gate = ACCEPTANCE_GATES[record['name']]
    if not gate['min_length_3d'] <= record['length_3d'] < gate['max_length_3d']:
        raise ValueError('Candidate failed measured length acceptance: ' + record['name'])
    if not 0 <= record['expanded'] <= gate['max_expanded']:
        raise ValueError('Candidate failed measured search-cost acceptance: ' + record['name'])
    if 'max_reversed_axis_yards' in gate and record['reversed_endpoint_axis_yards'] >= gate['max_reversed_axis_yards']:
        raise ValueError('Candidate still backtracks in the short bridge case: ' + record['name'])
    for index, key in ((0, 'start_surface_z'), (-1, 'goal_surface_z')):
        if key in gate and abs(record['points'][index][2] - gate[key]) > .05:
            raise ValueError('Candidate changed an endpoint surface: ' + record['name'])
    deviation = record['road_corridor_deviation']
    limit = gate['max_road_deviation_yards']
    if max(deviation['route_to_reference_max_yards'], deviation['reference_to_route_max_yards']) > limit:
        raise ValueError('Candidate left or skipped the measured road corridor: ' + record['name'])
    if 'bridge_windows' in record and not all(item['passed'] for item in record['bridge_windows']):
        raise ValueError('Candidate missed a bridge window or still backtracks there: ' + record['name'])
    diagnostic = record.get('road_diagnostics')
    if not diagnostic or diagnostic.get('result', {}).get('status') != 'success':
        raise ValueError('Candidate did not expose its successful query diagnostics: ' + record['name'])
    trace = diagnostic.get('road_trace', [])
    if not any(row.get('accepted') for row in trace) or any(row.get('mode') == 'remaining_native_fallback' for row in trace):
        raise ValueError('Candidate bypassed expected road-following validation: ' + record['name'])


def verify_native(payload, inputs, snapshots, report, native_factory=None):
    with candidate_modules(payload, inputs) as modules:
        nav, data, models, planner = (modules[n] for n in ('navmesh', 'data', 'models', 'planner'))
        original_native = nav.Native
        def pinned_native(path):
            if Path(path).resolve() != inputs['dll_path']:
                raise ValueError('Unexpected DLL selected during the candidate probe')
            if sha(read(inputs['dll_path'])) != DLL_SHA:
                raise ValueError('DLL changed before the native probe')
            return native_factory(original_native, path) if native_factory else original_native(path)
        nav.Native = pinned_native
        class SnapshotPack(nav.NavMeshPack):
            def read(self, entry):
                if (entry['path'], entry['bytes'], entry['sha256']) not in inputs['allowed_entries']:
                    raise ValueError('Manifest entry changed during the candidate probe')
                _, content = checked_asset(inputs['nav'], entry, snapshots)
                return content
        bundle = data.DataBundle.load(inputs['bundle_path'])
        coarse_planner = planner.Planner(bundle)
        report['native_queries'] = []
        for label, a, b in [('short_A_to_B', SHORT_A, SHORT_B),
                             ('short_B_to_A', SHORT_B, SHORT_A),
                             ('known_long_route', LONG_A, LONG_B),
                             ('known_long_reverse', LONG_B, LONG_A)]:
            say('验证道路引导路线 ' + label + ' ...')
            start, goal = models.Point(0, *a), models.Point(0, *b)
            guide = coarse_planner._plan_coarse(start, goal, allow_candidates=False)
            pack = SnapshotPack(inputs['manifest'])
            if pack.raw != inputs['raw']:
                raise ValueError('Manifest changed before the candidate query')
            # This isolated probe suppresses ordinary preview receipts. The
            # installer itself always keeps a full validation report.
            pack.record_diagnostics = False
            route = pack.plan(start, goal, guide=guide, blocked_edges=frozenset())
            record = route_record(label, route, guide, start, goal, inputs['raw']['identity'])
            record['road_diagnostics'] = getattr(pack, 'last_route_diagnostics', None)
            report['native_queries'].append(record)
            check_gate(record)
            say('  通过：%.4f 码，%d 个点，展开 %d 次' %
                (record['length_3d'], len(record['points']), record['expanded']))
    report['native_validation_passed'] = True


def receipts(root, payload):
    folder = beneath(root, root / 'runtime/route-quality-fixes')
    if not folder.is_dir():
        return []
    matches = []
    for path in sorted(folder.glob('*/repair.json'), reverse=True):
        path = beneath(root, path)
        try:
            record = json.loads(read(path))
            if (record.get('version') == VERSION and record.get('before_navmesh_sha256') == OLD_NAV_SHA
                    and record.get('after_navmesh_sha256') == sha(payload['navmesh'])
                    and record.get('helper_sha256') == sha(payload['road_corridor'])):
                before = read(beneath(root, path.parent / 'navmesh.before.py'))
                after = read(beneath(root, path.parent / 'navmesh.after.py'))
                helper = read(beneath(root, path.parent / 'road_corridor.after.py'))
                if sha(before) == OLD_NAV_SHA and after == payload['navmesh'] and helper == payload['road_corridor']:
                    matches.append((path, record, before))
        except (OSError, ValueError, TypeError):
            continue
    return matches


def rollback(root, payload, report):
    nav_path = beneath(root, root / 'adrive/navmesh.py')
    current = read(nav_path)
    if sha(current) == OLD_NAV_SHA:
        report['status'] = 'already_original'
        return
    helper_path = beneath(root, root / 'adrive/road_corridor.py')
    if current != payload['navmesh'] or read(helper_path) != payload['road_corridor']:
        raise ValueError('Installed code contains unrelated changes; rollback refused.')
    matches = receipts(root, payload)
    if not matches:
        raise ValueError('No exact backup for this installed code was found; rollback refused.')
    receipt, record, original = matches[0]
    snapshots = {}
    for path in (nav_path, helper_path, receipt, receipt.parent/'navmesh.before.py',
                 receipt.parent/'navmesh.after.py', receipt.parent/'road_corridor.after.py'):
        remember(snapshots, path, read(path))
    recheck(snapshots)
    atomic_replace(nav_path, original, expected=current)
    report.update(status='rolled_back', active_changed=True, repair_record=str(receipt),
                  backup_path=str(receipt.parent/'navmesh.before.py'), helper_retained=True)


def install(root, payload, report, report_path, native_factory):
    snapshots = {}
    say('阶段 1/3：核对规划源码、道路数据和导航版本 ...')
    inputs = preflight(root, payload, snapshots)
    report.update(manifest_sha256=sha(inputs['manifest_bytes']), pack_identity=inputs['raw']['identity'],
                  bundle_sha256=BUNDLE_SHA, native_library_sha256=DLL_SHA)
    say('阶段 2/3：使用当前 Windows DLL 验证候选规划逻辑 ...')
    report['status'] = 'native_validation_in_progress'
    atomic_replace(report_path, encoded(report))
    verify_native(payload, inputs, snapshots, report, native_factory)
    recheck(snapshots)
    if inputs['before'] == payload['navmesh']:
        saved = receipts(root, payload)
        if inputs['helper_before'] != payload['road_corridor'] or not saved:
            raise ValueError('Installed candidate lacks its exact helper or rollback backup.')
        report.update(status='already_installed_verified',
                      backup_path=str(saved[0][0].parent/'navmesh.before.py'))
        return
    say('阶段 3/3：保存精确备份，安装辅助模块，最后切换规划代码 ...')
    parent = beneath(root, root / 'runtime/route-quality-fixes')
    parent.mkdir(exist_ok=True)
    folder = parent / report['attempt_id']
    folder.mkdir()
    record = dict(version=VERSION, created_at_utc=report['started_at_utc'],
                  before_navmesh_sha256=OLD_NAV_SHA, after_navmesh_sha256=sha(payload['navmesh']),
                  helper_sha256=sha(payload['road_corridor']), helper_existed=inputs['helper_before'] is not None,
                  manifest_sha256=sha(inputs['manifest_bytes']), pack_identity=inputs['raw']['identity'],
                  bundle_sha256=BUNDLE_SHA, native_library_sha256=DLL_SHA,
                  report_path=report['report_path'], changes=['adrive/navmesh.py', 'adrive/road_corridor.py'],
                  untouched=['navigation manifest and identity', 'DLLs', 'navigation tile files'])
    durable_new(folder/'navmesh.before.py', inputs['before'])
    durable_new(folder/'navmesh.after.py', payload['navmesh'])
    durable_new(folder/'road_corridor.after.py', payload['road_corridor'])
    durable_new(folder/'repair.json', encoded(record))
    report.update(status='ready_to_install', repair_directory=str(folder),
                  backup_path=str(folder/'navmesh.before.py'))
    atomic_replace(report_path, encoded(report))
    recheck(snapshots)
    if inputs['helper_before'] is None:
        atomic_replace(inputs['helper'], payload['road_corridor'], expected=None)
        snapshots[inputs['helper']] = {'bytes': len(payload['road_corridor']), 'sha256': sha(payload['road_corridor'])}
        report['helper_created'] = True
    recheck(snapshots)
    atomic_replace(inputs['nav_source'], payload['navmesh'], expected=inputs['before'])
    report.update(status='installed_verified', active_changed=True)


def run(root, undo=False, *, _native_factory=None):
    root = Path(root).resolve()
    now = datetime.now(timezone.utc)
    attempt = now.strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8]
    report = dict(version=VERSION, attempt_id=attempt, started_at_utc=now.isoformat(),
                  project_root=str(root), operation='rollback' if undo else 'apply',
                  active_changed=False, native_validation_passed=False, status='started')
    report_path = None
    try:
        payload = payload_bytes()
        if not root.is_dir():
            raise ValueError('Project root does not exist')
        reports = beneath(root, root / 'runtime/route-quality-fix-reports')
        reports.mkdir(parents=True, exist_ok=True)
        report_path = reports / (attempt + '.json')
        report['report_path'] = str(report_path)
        atomic_replace(report_path, encoded(report))
        say('验证报告：' + str(report_path))
        if undo:
            rollback(root, payload, report)
        else:
            install(root, payload, report, report_path, _native_factory)
    except Exception as exc:
        report.update(status='failed', error=type(exc).__name__ + ': ' + str(exc))
        say('ERROR = ' + report['error'])
    report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
    if report_path:
        try:
            atomic_replace(report_path, encoded(report))
        except Exception as exc:
            report['report_write_error'] = type(exc).__name__ + ': ' + str(exc)
            say('报告写入失败：' + report['report_write_error'])
        say('REPORT = ' + str(report_path))
    if report.get('backup_path'):
        say('BACKUP = ' + report['backup_path'])
    return report


def main():
    say('ROUTE QUALITY FIX ' + VERSION + '：启动，正在准备校验。')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', type=Path, default=Path.cwd())
    parser.add_argument('--rollback', action='store_true', help='Restore only the exact saved navmesh.py.')
    args = parser.parse_args()
    if os.name != 'nt':
        parser.exit(1, 'This command targets the captured Windows installation.\n')
    report = run(args.project_root, args.rollback)
    if report['status'] == 'failed':
        say('ROUTE_QUALITY_FIX_FAILED')
        say('本次已切换规划代码；请按报告核对当前状态。' if report['active_changed']
            else '本次未切换规划代码；当前已有规划代码保持原样。')
        say('未完成更新；请发送上面的 ERROR 和报告文件。')
        return 1
    if args.rollback:
        say('ROUTE_QUALITY_FIX_ROLLED_BACK' if report['active_changed'] else 'ROUTE_QUALITY_FIX_ALREADY_ORIGINAL')
        say('已恢复原规划代码。' if report['active_changed'] else '当前已是原规划代码。')
    else:
        say('ROUTE_QUALITY_FIX_INSTALLED' if report['active_changed'] else 'ROUTE_QUALITY_FIX_ALREADY_INSTALLED_VERIFIED')
        say('规划逻辑已更新，当前 DLL 验路通过。' if report['active_changed'] else '同一修复已安装，当前 DLL 再次验路通过。')
        say('请重新启动 AutonomousDriving 导航程序，再预览原来的路线。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
