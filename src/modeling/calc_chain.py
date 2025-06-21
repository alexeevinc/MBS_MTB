import sys
import os
import numpy as np
import math as m
import matplotlib.pyplot as plt
from src.utils.datamanager import load_config

sys.path.append(os.path.abspath('C:/1_A/HSPF/repo_1'))

class CALC_CHAIN():
    def __init__(self):
        self.config = load_config()
        self.origin = np.array([0,0])
        self.pins_x = []
        self.pins_y = []
        self.pin_counter = 0
        self.pitch = self.config.rollers_dist
        self.lri12 = 1
        self.lrj12 = 1
        self.lri23 = 1
        self.lrj23 = -1
        self.lri34 = -1
        self.lrj34 = 1
        self.lri45 = 1
        self.lrj45 = 1
        self.lri56 = -1
        self.lrj56 = -1

    def trafo(self, angle):

        A = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]
        ])
        return A
    
    def angle_between(self, v1, v2):
        unit_v1 = v1 / np.linalg.norm(v1)
        unit_v2 = v2 / np.linalg.norm(v2)
        dot_product = np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0)
        angle_rad = np.arccos(dot_product)
        angle_deg = np.degrees(angle_rad)
        return angle_deg

    def calc_signs(self, lri, lrj):
                
        if lri==lrj:
            sign1 = -1
        else:
            sign1 = 1

        sign2 = 1

        if ((lri==-1 and lrj==1) or (lri==1 and lrj==1)):
            sign2 = 1
        elif ((lri==1 and lrj==-1) or (lri==-1 and lrj==-1)):
            sign2=-1

        if lri==lrj:
            sign3 = 1
        else:
            sign3 = -1        

        if ((lri==1 and lrj==1) or (lri==-1 and lrj==1)):
            sign4 = -1
        else:
            sign4 = 1

        return sign1, sign2, sign3, sign4
    
    def append4export(self, x_list, y_list):
        self.pins_x.extend(x_list)
        self.pins_y.extend(y_list)

    def export_xy(self):
        with open('src/data/pins_xy.txt', 'w') as f:
            for x, y in zip(self.pins_x, self.pins_y):
                f.write(f"{x}\t{y}\n")

    def calc_strand(self, loc_i, loc_j, rad_i, rad_j, lri, lrj):
        sign1, sign2, sign3, sign4 = self.calc_signs(lri, lrj)
        self.pitch = self.config.rollers_dist
        # ri = np.array(self.config.sprocket_main_loc)
        ri = np.array([loc_i[1], loc_i[2]])
        rj = np.array([loc_j[1], loc_j[2]])
        dc = np.array(rj - ri)
        dc_len = np.linalg.norm(dc)
        dp = m.sqrt(dc_len**2 - (rad_j + sign1*rad_i)**2)
        uc = - dc/dc_len

        beta = sign2*(m.pi/2 - m.atan((rad_j+sign1*rad_i)/dp))
        A_rot_beta = self.trafo(beta)
        n = A_rot_beta @ uc
        
        n_ci = sign3*rad_i*n
        n_cj = rad_j*n
        r_pi = ri + n_ci
        r_pj = rj + n_cj
        n_pins = int(dp/self.pitch)

        phi_1a = m.acos(((n_pins*self.pitch)**2+np.linalg.norm(rj-r_pi)**2-rad_j**2)/(2*n_pins*self.pitch*np.linalg.norm(rj-r_pi)))
        print(m.degrees(phi_1a))
        phi_1b = m.acos((((n_pins+1)*self.pitch)**2+np.linalg.norm(rj-r_pi)**2-rad_j**2)/(2*(n_pins+1)*self.pitch*np.linalg.norm(rj-r_pi)))
        print(m.degrees(phi_1b))

        if phi_1a > phi_1b:
            phi = phi_1a
            n_pins += 1
        else:
            phi = phi_1b
            n_pins += 2

        
        phi = sign4*phi
        A_rot_phi = self.trafo(phi)
        up = A_rot_phi @ ((rj-r_pi)/(np.linalg.norm(rj-r_pi)))

        pins_x_strand1 = []
        pins_y_strand1 = []
        # n_pins += 1

        for k in range(0,n_pins):
            r_pin = r_pi + self.pitch*k*up
            pins_x_strand1.append(r_pin[0])
            pins_y_strand1.append(r_pin[1])

        return ri, rj, dc, uc, n, n_ci, n_cj, r_pi, r_pj, up, pins_x_strand1, pins_y_strand1, rad_i, rad_j
    
    def calc_strand2(self, loc_i, loc_j, rad_i, rad_j, lri, lrj, r_pi_new):
        sign1, sign2, sign3, sign4 = self.calc_signs(lri, lrj)
        
        # ri = np.array(self.config.sprocket_main_loc)
        ri = np.array([loc_i[1], loc_i[2]])
        rj = np.array([loc_j[1], loc_j[2]])
        dc = np.array(rj - ri)
        dc_len = np.linalg.norm(dc)

        dp = m.sqrt(dc_len**2 - (rad_j + sign1*rad_i)**2)
        uc = - dc/dc_len

        beta = sign2*(m.pi/2 - m.atan((rad_j+sign1*rad_i)/dp))
        A_rot_beta = self.trafo(beta)
        n = A_rot_beta @ uc
        
        n_ci = sign3*rad_i*n
        n_cj = rad_j*n
        r_pi = ri + n_ci
        r_pj = rj + n_cj
        n_pins = int(dp/self.pitch)

        a = rj-np.array(r_pi_new)
        b = rj-r_pi

        phi_1a = m.acos(((n_pins*self.pitch)**2+np.linalg.norm(a)**2-rad_j**2)/(2*n_pins*self.pitch*np.linalg.norm(a)))
        print(m.degrees(phi_1a))
        phi_1b = m.acos((((n_pins+1)*self.pitch)**2+np.linalg.norm(a)**2-rad_j**2)/(2*(n_pins+1)*self.pitch*np.linalg.norm(a)))
        print(m.degrees(phi_1b))

        if phi_1a > phi_1b:
            phi = phi_1a
            n_pins += 1
        else:
            phi = phi_1b
            n_pins += 2
  
        phi = sign4*phi
        A_rot_phi = self.trafo(phi)
        up = A_rot_phi @ ((rj-np.array(r_pi_new))/(np.linalg.norm(rj-np.array(r_pi_new))))

        pins_x_strand = []
        pins_y_strand = []
        
        for k in range(0,n_pins):
            r_pin = np.array(r_pi_new) + self.pitch*k*up
            pins_x_strand.append(r_pin[0])
            pins_y_strand.append(r_pin[1])

        return ri, rj, dc, uc, n, n_ci, n_cj, r_pi, r_pj, up, pins_x_strand, pins_y_strand, rad_i, rad_j

    def calc_wrap(self, r_pi_j, r_pj_j, rj, r_pj, rad_j, pitch, lri, lrj):
        s_pi = r_pi_j - rj
        s_pj = r_pj_j - rj

        psi = m.acos(s_pj.T@s_pi/rad_j**2)
        print('psi='+str(m.degrees(psi)))
        alpha_j = m.radians(pitch)
        n = int(psi/alpha_j)
        delta_1a = psi-n*alpha_j
        delta_1b = (n+1)*alpha_j - psi
        if delta_1a > delta_1b:
            n += 1
        u_pj = ((r_pj_j-rj)/np.linalg.norm(r_pj_j-rj))

        pins_x_wrap = []
        pins_y_wrap = []
        for k in range(0,n+1):
            self.pin_counter += 1
            phi_k = lrj*k*alpha_j
            A_rot_phi_k = self.trafo(phi_k)
            r_pin = rj + A_rot_phi_k @ u_pj*rad_j
            pins_x_wrap.append(r_pin[0])
            pins_y_wrap.append(r_pin[1])
        self.pin_counter -= 2
        return pins_x_wrap, pins_y_wrap
     
    def calc_slack(self, pn, pm):
        rn = np.array(pn)
        rm = np.array(pm)
        l = rn - rm
        l_sec = np.linalg.norm(l)
        # print("length L = "+str(l_sec))
        n1 = int(l_sec/(2*self.pitch))+1
        n2 = int(l_sec/(2*self.pitch))+1
        l1 = n1*self.pitch
        l2 = n2*self.pitch
        alpha = m.acos((l1**2+l_sec**2-l2**2)/(2*l1*l_sec))
        a1 = l1*m.cos(alpha)
        a2 = l_sec - a1
        # a1 = l_sec**2 - l2**2 - l1**2/(2*l_sec)
        lc = m.sqrt(l1**2-a1**2)
        u = ((rn-rm)/(np.linalg.norm(rn-rm)))
        A = self.trafo(m.radians(90))
        v = A @ u
        rc = rm + a1*u + v*lc # scale lc for maximum slack deviation
        v1 = ((rc-rm)/(np.linalg.norm(rc-rm)))
        v2 = ((rn-rc)/(np.linalg.norm(rn-rc)))
        pins_x_slack = []
        pins_y_slack = []
        n1 = 19
        for k in range(1,n1):
            # self.pin_counter += 1
            r_pin1 = rm + k*v1*self.pitch
            pins_x_slack.append(r_pin1[0])
            pins_y_slack.append(r_pin1[1])

        n2 = 19
        for k in range(1,n2):
            # self.pin_counter += 1
            r_pin1 = rc + k*v2*self.pitch
            pins_x_slack.append(r_pin1[0])
            pins_y_slack.append(r_pin1[1])

        return rn, rm, u, v, a1, a2, rc, pins_x_slack, pins_y_slack

    def visualize_strand(self, axes, ri, rj, dc, uc, n, n_ci, n_cj, r_pi, r_pj, up, pins_x_strand, pins_y_strand, rad_i, rad_j, color_j):
        w = 0.0015
        # plt.quiver(*self.origin, *ri, angles='xy', scale_units='xy', scale=1, color='red', width=w)
        # plt.quiver(*self.origin, *rj, angles='xy', scale_units='xy', scale=1, color=color_j, width=w)
        # plt.quiver(*ri, *dc, angles='xy', scale_units='xy', scale=1, color='black', width=w)
        # plt.quiver(*rj, *uc*20, angles='xy', scale_units='xy', scale=1, color='green', width=w)
        # plt.quiver(*rj, *n*20, angles='xy', scale_units='xy', scale=1, color='orange', width=w)
        # plt.quiver(*rj, *n_cj, angles='xy', scale_units='xy', scale=1, color='black', width=w)
        # plt.quiver(*ri, *n_ci, angles='xy', scale_units='xy', scale=1, color='black', width=w)
        plt.quiver(*r_pi, *up*10, angles='xy', scale_units='xy', scale=1, color='magenta', width=w)
        plt.plot([r_pi[0], r_pj[0]], [r_pi[1], r_pj[1]], 'grey')
        plt.scatter([r_pi[0], r_pj[0]], [r_pi[1], r_pj[1]], color='green', s=10)
        plt.scatter(pins_x_strand, pins_y_strand, color='blue', s=30)
        circle_j = plt.Circle((rj[0], rj[1]), rad_j, edgecolor=color_j, facecolor='none', linewidth=1)
        axes.add_artist(circle_j)
        self.pin_counter += len(pins_x_strand)

    def visuslize_deraiileur(self):
        a = 1

    def calc_jockeys(self, angle_cage_deg, angle_der_deg, angle_jl_deg, uc):
        def getdir_and_rotate(ri, rj, angle_deg):
            angle = m.radians(angle_deg)
            A = self.trafo(angle)
            dc = np.array(rj - ri)
            dc_len = np.linalg.norm(dc)
            uc = - dc/dc_len
            uc_rot = A @ uc
            return uc_rot
        
        r_main = np.array([self.config.sprocket_main_loc[1], self.config.sprocket_main_loc[2]])
        r_cas = np.array([self.config.cassette_loc[1], self.config.cassette_loc[2]])
        r_ju = np.array([self.config.jockey_upr_loc_gl[1], self.config.jockey_upr_loc_gl[2]])
        r_jl = np.array([self.config.jockey_lwr_loc_gl[1], self.config.jockey_lwr_loc_gl[2]])
        r_kn = np.array([self.config.knuckle_loc[1], self.config.knuckle_loc[2]])

        r_kn_lcl = r_cas - r_kn
        der_len = np.linalg.norm(r_kn_lcl) # derilleur length

        uc_kn = getdir_and_rotate(r_main, r_cas, angle_der_deg)
        r_kn_loc_rot = uc_kn*der_len # derailleur rotated

        r_ju_lcl = r_ju - r_kn
        r_jl_lcl = r_jl - r_kn

        jl_len = np.linalg.norm(r_jl_lcl) # distance knuckle to jockey lwr
        uc_jl = getdir_and_rotate(r_cas, r_kn, angle_jl_deg)
        r_jl_loc_rot = uc_jl*jl_len

        uc_ju = getdir_and_rotate(r_cas+r_kn_loc_rot+r_jl_loc_rot,r_cas+r_kn_loc_rot,  self.config.jockey_upr_2_kn_deg)
        r_ju_loc_rot = uc_ju*self.config.jockey_upr_2_kn_len
        
        angle_cage = m.radians(angle_cage_deg)
        A_cage = self.trafo(angle_cage)
        r_ju_lcl_rot = A_cage @ r_ju_lcl 
        r_jl_lcl_rot = A_cage @ r_jl_lcl 

        r_ju_rot = r_ju_lcl_rot + r_kn
        r_jl_rot = r_jl_lcl_rot + r_kn
        l1 = [0]
        l2 = [0]
        l1.extend(r_ju_rot)
        l2.extend(r_jl_rot)

        self.kn_gl_rot = [0]
        self.jl_gl_rot = [0]
        self.ju_gl_rot = [0]

        self.kn_gl_rot.extend(r_cas + r_kn_loc_rot)
        self.jl_gl_rot.extend(r_cas + r_kn_loc_rot + r_jl_loc_rot)
        self.ju_gl_rot.extend(r_cas + r_kn_loc_rot + r_ju_loc_rot)

        return l1, l2, r_kn, r_kn_loc_rot, r_jl_loc_rot, r_ju_loc_rot

    def visualize_slack(self, axes, rn, rm, u, v, a1, a2, rc, pins_x, pins_y):
        # self.append4export(pins_x[::-1], pins_y[::-1])
        w = 0.0015
        plt.quiver(*self.origin, *rn, angles='xy', scale_units='xy', scale=1, color='red', width=w)
        plt.quiver(*self.origin, *rm, angles='xy', scale_units='xy', scale=1, color='blue', width=w)
        plt.quiver(*rm, *u, angles='xy', scale_units='xy', scale=1, color='green', width=w)
        plt.quiver(*u*a1+rm, *v, angles='xy', scale_units='xy', scale=1, color='magenta', width=w)
        plt.quiver(*self.origin, *rc, angles='xy', scale_units='xy', scale=1, color='magenta', width=w)
        plt.scatter(pins_x, pins_y, color='green', s=30)

    def calc_n_visualize(self):
        ### CALCULATION ###

        # STRAND 1
        ri, rj, dc, uc, n, n_ci, n_cj, r_pi, r_pj, up, pins_x_strand1, pins_y_strand1, rad_i, rad_j = self.calc_strand(
            self.config.sprocket_upr_loc, self.config.cassette_loc, self.config.sprocket_upr_roll_r, self.config.cassette_rad, self.lri12, self.lrj12)
        
        jockey_upr_loc, jockey_lwr_loc, knuckle_gl, knuckle_loc_rot, jl_loc_rot, ju_loc_rot = self.calc_jockeys(angle_cage_deg=0,
                                                                             angle_der_deg=-47.9, angle_jl_deg=95.6, uc=uc) # for cassette 4

        # STRAND 2
        ri2, rj2, dc2, uc2, n2, n_ci2, n_cj2, r_pi2, r_pj2, up2, pins_x_strand2, pins_y_strand2, rad_i2, rad_j2= self.calc_strand(
            self.config.cassette_loc, self.ju_gl_rot, self.config.cassette_rad, self.config.jockey_upr_rad, self.lri23, self.lrj23)

        jlgl = rj+knuckle_loc_rot+jl_loc_rot
        jugl = rj+knuckle_loc_rot+ju_loc_rot
        kngl = rj+knuckle_loc_rot
        
        # WRAP 1-2
        pins_x_wrap12, pins_y_wrap12 = self.calc_wrap([pins_x_strand2[0], pins_y_strand2[0]], [pins_x_strand1[-1], pins_y_strand1[-1]],
                                                       rj, r_pj, rad_j, self.config.cassette_pitch_deg, self.lri12, self.lrj12)

        # STRAND 2 CORRECTED
        ri22, rj22, dc22, uc22, n22, n_ci22, n_cj22, r_pi22, r_pj22, up22, pins_x_strand22, pins_y_strand22, rad_i22, rad_j22= self.calc_strand2(
            self.config.cassette_loc, self.ju_gl_rot, self.config.cassette_rad, self.config.jockey_upr_rad,
              self.lri23, self.lrj23, [pins_x_wrap12[-1], pins_y_wrap12[-1]])
        
        # STRAND 3
        ri3, rj3, dc3, uc3, n3, n_ci3, n_cj3, r_pi3, r_pj3, up3, pins_x_strand3, pins_y_strand3, rad_i3, rad_j3= self.calc_strand(
            self.ju_gl_rot, self.jl_gl_rot, self.config.jockey_upr_rad, self.config.jockey_lwr_rad, self.lri34, self.lrj34)
        
        # WRAP 2-3
        pins_x_wrap23, pins_y_wrap23 = self.calc_wrap([pins_x_strand3[0], pins_y_strand3[0]], [pins_x_strand22[-1], pins_y_strand22[-1]], 
                                                      rj22, r_pj22, rad_j22, self.config.jockey_upr_pitch_deg, self.lri23, self.lrj23) 

        # STRAND 3 CORRECTED
        ri33, rj33, dc33, uc33, n33, n_ci33, n_cj33, r_pi33, r_pj33, up33, pins_x_strand33, pins_y_strand33, rad_i33, rad_j33= self.calc_strand2(
            self.ju_gl_rot, self.jl_gl_rot, self.config.jockey_upr_rad, self.config.jockey_lwr_rad,
              self.lri34, self.lrj34, [pins_x_wrap23[-1], pins_y_wrap23[-1]])
        
        # STRAND 4
        ri4, rj4, dc4, uc4, n4, n_ci4, n_cj4, r_pi4, r_pj4, up4, pins_x_strand4, pins_y_strand4, rad_i4, rad_j4= self.calc_strand(
            self.jl_gl_rot, self.config.sprocket_main_loc, self.config.jockey_lwr_rad, self.config.sprocket_main_rad, self.lri45, self.lrj45)
        
        # WRAP 3-4
        pins_x_wrap34, pins_y_wrap34 = self.calc_wrap([pins_x_strand4[0], pins_y_strand4[0]], [pins_x_strand33[-1], pins_y_strand33[-1]], 
                                                      rj33, r_pj33, rad_j33, self.config.jockey_lwr_pitch_deg, self.lri34, self.lrj34) 
        
        # STRAND 4 CORRECTED
        ri44, rj44, dc44, uc44, n44, n_ci44, n_cj44, r_pi44, r_pj44, up44, pins_x_strand44, pins_y_strand44, rad_i44, rad_j44 = self.calc_strand2(
            self.jl_gl_rot, self.config.sprocket_main_loc, self.config.jockey_lwr_rad, self.config.sprocket_main_rad, 
            self.lri45, self.lrj45, [pins_x_wrap34[-1], pins_y_wrap34[-1]])
                
        # STRAND 5
        ri5, rj5, dc5, uc5, n5, n_ci5, n_cj5, r_pi5, r_pj5, up5, pins_x_strand5, pins_y_strand5, rad_i5, rad_j5= self.calc_strand(
            self.config.sprocket_upr_loc, self.config.sprocket_main_loc, self.config.sprocket_upr_roll_r, self.config.sprocket_main_rad, self.lri56, self.lrj56)
        
        # WRAP 5-1
        pins_x_wrap51, pins_y_wrap51 = self.calc_wrap([pins_x_strand5[0], pins_y_strand5[0]], [pins_x_strand1[0], pins_y_strand1[0]],
                                                      ri, r_pi, rad_i, self.config.sprocket_upr_pitch_deg, self.lri56, self.lrj56) 
        
        # STRAND 5 CORRECTED
        ri55, rj55, dc55, uc55, n55, n_ci55, n_cj55, r_pi55, r_pj55, up55, pins_x_strand55, pins_y_strand55, rad_i55, rad_j55 = self.calc_strand2(
            self.config.sprocket_upr_loc, self.config.sprocket_main_loc, self.config.sprocket_upr_roll_r, self.config.sprocket_main_rad, self.lri56, self.lrj56,
            [pins_x_wrap51[-1], pins_y_wrap51[-1]])  

        # # WRAP 4-5
        pins_x_wrap45, pins_y_wrap45 = self.calc_wrap([pins_x_strand44[-1], pins_y_strand44[-1]], [pins_x_strand55[-1], pins_y_strand55[-1]], 
                                                      rj44, r_pj44, rad_j44, self.config.sprocket_main_pitch_deg, self.lri45, -self.lrj45)    
        
        rn, rm, u, v, a1, a2, rc, pins_x_slack, pins_y_slack = self.calc_slack([pins_x_wrap34[-1], pins_y_wrap34[-1]], [pins_x_wrap45[-2], pins_y_wrap45[-2]])


        # EXPORT COORDINATES OF PINS
        print("export list len = "+str(len(self.pins_x))) 
        
        ###
        ### VISUALIZATION ###
        ###
        fig, axes = plt.subplots(figsize=(20, 10))
        w = 0.0015
        # DERAIILEUR
        knuckle_loc = knuckle_gl-rj
        plt.quiver(*rj, *knuckle_loc, angles='xy', scale_units='xy', scale=1, color='black', width=w) # cassette to knuckle
        plt.quiver(*knuckle_gl, *(([jockey_lwr_loc[1],jockey_lwr_loc[2]]) - knuckle_gl), angles='xy', scale_units='xy', scale=1, color='black', width=w) 
        plt.quiver(*self.origin, *rj, angles='xy', scale_units='xy', scale=1, color='black', width=w)
        plt.quiver(*rj, *knuckle_loc_rot, angles='xy', scale_units='xy', scale=1, color='green', width=w)
        plt.quiver(*rj+knuckle_loc_rot, *jl_loc_rot, angles='xy', scale_units='xy', scale=1, color='green', width=w)
        plt.quiver(*rj+knuckle_loc_rot, *ju_loc_rot, angles='xy', scale_units='xy', scale=1, color='green', width=w)
        a1 = 180 - self.angle_between(rj, knuckle_loc)
        # FIRST SPROCKET
        circle_i1 = plt.Circle((ri[0], ri[1]), rad_i, edgecolor='red', facecolor='none', linewidth=1)
        axes.add_artist(circle_i1)
                
        # STRAND 1
        self.visualize_strand(axes, ri, rj, dc, uc, n, n_ci, n_cj, r_pi, r_pj, up, pins_x_strand1, pins_y_strand1, rad_i, rad_j, 'blue')
        self.append4export(pins_x_strand1, pins_y_strand1)

        # WRAP 1-2
        plt.scatter(pins_x_wrap12[1:-1], pins_y_wrap12[1:-1], color='red', s=30)
        self.append4export(pins_x_wrap12[1:-1], pins_y_wrap12[1:-1])

        # STRAND 2
        # self.visualize_strand(axes, ri2, rj2, dc2, uc2, n2, n_ci2, n_cj2, r_pi2, r_pj2, up2, pins_x_strand2, pins_y_strand2, rad_i2, rad_j2)
        self.visualize_strand(axes, ri22, rj22, dc22, uc22, n22, n_ci22, n_cj22, r_pi22, r_pj22, up22, pins_x_strand22, pins_y_strand22, rad_i22, rad_j22, 'green')
        self.append4export(pins_x_strand22, pins_y_strand22)
        # WRAP 2-3
        plt.scatter(pins_x_wrap23[1:-1], pins_y_wrap23[1:-1], color='red', s=30)
        self.append4export(pins_x_wrap23[1:-1], pins_y_wrap23[1:-1])

        # STRAND 3
        # self.visualize_strand(axes, ri3, rj3, dc3, uc3, n3, n_ci3, n_cj3, r_pi3, r_pj3, up3, pins_x_strand3, pins_y_strand3, rad_i3, rad_j3, 'magenta')
        self.visualize_strand(axes, ri33, rj33, dc33, uc33, n33, n_ci33, n_cj33, r_pi33, r_pj33, up33, pins_x_strand33, pins_y_strand33, rad_i33, rad_j33, 'magenta')
        self.append4export(pins_x_strand33, pins_y_strand33)
        # WRAP 3-4
        plt.scatter(pins_x_wrap34[1:-1], pins_y_wrap34[1:-1], color='red', s=30)  
        self.append4export(pins_x_wrap34[1:-1], pins_y_wrap34[1:-1])

        # SLACK
        self.visualize_slack(axes, rn, rm, u, v, a1, a2, rc, pins_x_slack, pins_y_slack)

        # STRAND 4
        # self.visualize_strand(axes, ri4, rj4, dc4, uc4, n4, n_ci4, n_cj4, r_pi4, r_pj4, up4, pins_x_strand4, pins_y_strand4, rad_i4, rad_j4, 'brown')
        self.visualize_strand(axes, ri44, rj44, dc44, uc44, n44, n_ci44, n_cj44, r_pi44, r_pj44, up44, pins_x_strand44, pins_y_strand44, rad_i44, rad_j44, 'brown')
        self.append4export(pins_x_strand44, pins_y_strand44)

        # WRAP 4-5
        x45 = pins_x_wrap45[1:-1]
        y45 = pins_y_wrap45[1:-1]
        plt.scatter(x45, y45, color='red', s=30)   
        self.append4export(x45[::-1], y45[::-1])

        # STRAND 5
        # self.visualize_strand(axes, ri5, rj5, dc5, uc5, n5, n_ci5, n_cj5, r_pi5, r_pj5, up5, pins_x_strand5, pins_y_strand5, rad_i5, rad_j5, 'brown')
        self.visualize_strand(axes, ri55, rj55, dc55, uc55, n55, n_ci55, n_cj55, r_pi55, r_pj55, up55, pins_x_strand55, pins_y_strand55, rad_i55, rad_j55, 'brown')
        self.append4export(pins_x_strand55[::-1], pins_y_strand55[::-1])
        # WRAP 5-1
        x51 =  pins_x_wrap51[1:-1]
        y51 = pins_y_wrap51[1:-1]
        plt.scatter(x51, y51, color='red', s=30)
        self.append4export(x51[::-1], y51[::-1])

        plt.xlim(-520, 100)
        plt.ylim(-250, 180)

        plt.grid()

        plt.axhline(0, color='black', linewidth=0.1)
        plt.axvline(0, color='black', linewidth=0.1)
        plt.gca().set_aspect('equal', adjustable='box')

        plt.title('2D Chain')
        plt.xlabel('X (Adm Y)')
        plt.ylabel('Y (Adm Z)')

        print("number of pins = "+ str(self.pin_counter))
        plt.show()
        

