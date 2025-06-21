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

    def trafo(self, angle):
        A = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]
        ])
        return A


    def calc_strand(self, loc_i, loc_j, rad_i, rad_j, lri, lrj):
        pitch = self.config.rollers_dist
        # ri = np.array(self.config.sprocket_main_loc)
        ri = np.array([loc_i[1], loc_i[2]])
        rj = np.array([loc_j[1], loc_j[2]])
        dc = np.array(rj - ri)
        dc_len = np.linalg.norm(dc)

        if lri==lrj:
            sign = -1
        else:
            sign = 1
        dp = m.sqrt(dc_len**2 - (rad_j + sign*rad_i)**2)
        uc = - dc/dc_len

        sign2 = 1

        if ((lri==-1 and lrj==1) or (lri==1 and lrj==1)or (lri==-1 and lrj==-1)):
            sign2 = 1
        elif ((lri==1 and lrj==-1) or (lri==1 and lrj==-1)):
            sign2=-1

        beta = sign2*(m.pi/2 - m.atan((rad_j+sign*rad_i)/dp))
        A_rot_beta = self.trafo(beta)
        n = A_rot_beta @ uc
        
        if lri==lrj:
            sign3 = 1
        else:
            sign3 = -1

        n_ci = sign3*rad_i*n
        n_cj = rad_j*n
        r_pi = ri + n_ci
        r_pj = rj + n_cj
        n_pins = int(dp/pitch)

        phi_1a = m.acos(((n_pins*pitch)**2+np.linalg.norm(rj-r_pi)**2-rad_j**2)/(2*n_pins*pitch*np.linalg.norm(rj-r_pi)))
        print(m.degrees(phi_1a))
        phi_1b = m.acos((((n_pins+1)*pitch)**2+np.linalg.norm(rj-r_pi)**2-rad_j**2)/(2*(n_pins+1)*pitch*np.linalg.norm(rj-r_pi)))
        print(m.degrees(phi_1b))

        if phi_1a > phi_1b:
            n_pins += 1
        
        if lri==lrj:
            sign4 = -1
        else:
            sign4 = 1

        phi = sign4*phi_1a
        A_rot_phi = self.trafo(phi)
        up = A_rot_phi @ ((rj-r_pi)/(np.linalg.norm(rj-r_pi)))

        pins_x_strand1 = []
        pins_y_strand1 = []
        
        for k in range(0,n_pins):
            r_pin = r_pi + pitch*k*up
            pins_x_strand1.append(r_pin[0])
            pins_y_strand1.append(r_pin[1])

        return ri, rj, dc, uc, n, n_ci, n_cj, r_pi, r_pj, up, pins_x_strand1, pins_y_strand1, rad_i, rad_j
    

    def calc_strand2(self, loc_i, loc_j, rad_i, rad_j, lri, lrj, r_pi_new):
        pitch = self.config.rollers_dist
        # ri = np.array(self.config.sprocket_main_loc)
        ri = np.array([loc_i[1], loc_i[2]])
        rj = np.array([loc_j[1], loc_j[2]])
        dc = np.array(rj - ri)
        dc_len = np.linalg.norm(dc)

        if lri==lrj:
            sign = -1
        else:
            sign = 1
        dp = m.sqrt(dc_len**2 - (rad_j + sign*rad_i)**2)
        uc = - dc/dc_len

        sign2 = 1

        if ((lri==-1 and lrj==1) or (lri==1 and lrj==1)or (lri==-1 and lrj==-1)):
            sign2 = 1
        elif ((lri==1 and lrj==-1) or (lri==1 and lrj==-1)):
            sign2=-1

        beta = sign2*(m.pi/2 - m.atan((rad_j+sign*rad_i)/dp))
        A_rot_beta = self.trafo(beta)
        n = A_rot_beta @ uc
        
        if lri==lrj:
            sign3 = 1
        else:
            sign3 = -1

        n_ci = sign3*rad_i*n
        n_cj = rad_j*n
        r_pi = ri + n_ci
        r_pj = rj + n_cj
        n_pins = int(dp/pitch)

        a = rj-np.array(r_pi_new)
        b = rj-r_pi

        phi_1a = m.acos(((n_pins*pitch)**2+np.linalg.norm(a)**2-rad_j**2)/(2*n_pins*pitch*np.linalg.norm(a)))
        print(m.degrees(phi_1a))
        phi_1b = m.acos((((n_pins+1)*pitch)**2+np.linalg.norm(a)**2-rad_j**2)/(2*(n_pins+1)*pitch*np.linalg.norm(a)))
        print(m.degrees(phi_1b))

        if phi_1a > phi_1b:
            n_pins += 1
        
        if lri==lrj:
            sign4 = -1
        else:
            sign4 = 1

        phi = sign4*phi_1a
        A_rot_phi = self.trafo(phi)
        up = A_rot_phi @ ((rj-np.array(r_pi_new))/(np.linalg.norm(rj-np.array(r_pi_new))))
        # up_large =  500 * up

        pins_x_strand1 = []
        pins_y_strand1 = []
        
        for k in range(0,n_pins):
            r_pin = np.array(r_pi_new) + pitch*k*up
            pins_x_strand1.append(r_pin[0])
            pins_y_strand1.append(r_pin[1])

        return ri, rj, dc, uc, n, n_ci, n_cj, r_pi, r_pj, up, pins_x_strand1, pins_y_strand1, rad_i, rad_j

    def calc_wrap(self, r_pi_j, r_pj_j, rj, r_pj, rad_j):
        s_pi = r_pi_j - rj
        s_pj = r_pj_j - rj

        psi = m.acos(s_pj.T@s_pi/rad_j**2)
        print('psi='+str(m.degrees(psi)))
        alpha_j = m.radians(self.config.cassette_pitch_deg)
        n = int(psi/alpha_j)
        delta_1a = psi-n*alpha_j
        delta_1b = (n+1)*alpha_j - psi
        if delta_1a > delta_1b:
            n += 1
        u_pj = ((r_pj_j-rj)/np.linalg.norm(r_pj_j-rj))

        pins_x_wrap = []
        pins_y_wrap = []
        for k in range(0,n+1):
            phi_k = k*alpha_j
            A_rot_phi_k = np.array([
            [np.cos(phi_k), -np.sin(phi_k)],
            [np.sin(phi_k),  np.cos(phi_k)]
            ])
            r_pin = rj + A_rot_phi_k @ u_pj*rad_j
            pins_x_wrap.append(r_pin[0])
            pins_y_wrap.append(r_pin[1])
        return pins_x_wrap, pins_y_wrap

    def visualize_strand(self, axes, ri, rj, dc, uc, n, n_ci, n_cj, r_pi, r_pj, up, pins_x_strand, pins_y_strand, rad_i, rad_j, color_j):
        w = 0.0015
        # STRAND 1
        plt.quiver(*self.origin, *ri, angles='xy', scale_units='xy', scale=1, color='red', width=w)
        plt.quiver(*self.origin, *rj, angles='xy', scale_units='xy', scale=1, color=color_j, width=w)
        plt.quiver(*ri, *dc, angles='xy', scale_units='xy', scale=1, color='black', width=w)
        plt.quiver(*rj, *uc*20, angles='xy', scale_units='xy', scale=1, color='green', width=w)
        plt.quiver(*rj, *n*20, angles='xy', scale_units='xy', scale=1, color='orange', width=w)
        plt.quiver(*rj, *n_cj, angles='xy', scale_units='xy', scale=1, color='black', width=w)
        plt.quiver(*ri, *n_ci, angles='xy', scale_units='xy', scale=1, color='black', width=w)
        plt.quiver(*r_pi, *up*20, angles='xy', scale_units='xy', scale=1, color='magenta', width=w)
        plt.plot([r_pi[0], r_pj[0]], [r_pi[1], r_pj[1]], 'grey')
        plt.scatter([r_pi[0], r_pj[0]], [r_pi[1], r_pj[1]], color='green', s=10)
        plt.scatter(pins_x_strand, pins_y_strand, color='blue', s=30)
        circle_j = plt.Circle((rj[0], rj[1]), rad_j, edgecolor=color_j, facecolor='none', linewidth=1)
        axes.add_artist(circle_j)

    def calc_n_visualize(self):
        # STRAND 1
        ri, rj, dc, uc, n, n_ci, n_cj, r_pi, r_pj, up, pins_x_strand1, pins_y_strand1, rad_i, rad_j = self.calc_strand(
            self.config.sprocket_upr_loc, self.config.cassette_loc, self.config.sprocket_upr_roll_r, self.config.cassette_rad, 1, 1)
        
        # STRAND 1
        ri2, rj2, dc2, uc2, n2, n_ci2, n_cj2, r_pi2, r_pj2, up2, pins_x_strand2, pins_y_strand2, rad_i2, rad_j2= self.calc_strand(
            self.config.cassette_loc, self.config.jockey_upr_loc_gl, self.config.cassette_rad, self.config.jockey_upr_rad, 1, -1)
        
        pins_x_wrap, pins_y_wrap = self.calc_wrap([pins_x_strand2[0], pins_y_strand2[0]], [pins_x_strand1[-1], pins_y_strand1[-1]], rj, r_pj, rad_j)

        ri22, rj22, dc22, uc22, n22, n_ci22, n_cj22, r_pi22, r_pj22, up22, pins_x_strand22, pins_y_strand22, rad_i22, rad_j22= self.calc_strand2(
            self.config.cassette_loc, self.config.jockey_upr_loc_gl, self.config.cassette_rad, self.config.jockey_upr_rad, 1, -1, [pins_x_wrap[-1], pins_y_wrap[-1]])
        
        # STRAND 1
        ri3, rj3, dc3, uc3, n3, n_ci3, n_cj3, r_pi3, r_pj3, up3, pins_x_strand3, pins_y_strand3, rad_i3, rad_j3= self.calc_strand(
            self.config.jockey_upr_loc_gl, self.config.jockey_lwr_loc_gl, self.config.jockey_upr_rad, self.config.jockey_upr_rad, -1, 1)
        
        fig, axes = plt.subplots(figsize=(20, 10))
        circle_i1 = plt.Circle((ri[0], ri[1]), rad_i, edgecolor='red', facecolor='none', linewidth=1)
        axes.add_artist(circle_i1)
        
        ### VISUALIZATION ###
        w = 0.0015
        # # STRAND 1
        self.visualize_strand(axes, ri, rj, dc, uc, n, n_ci, n_cj, r_pi, r_pj, up, pins_x_strand1, pins_y_strand1, rad_i, rad_j, 'blue')

        # STRAND 2
        # plt.scatter(pins_x_strand2, pins_y_strand2, color='blue', s=20)
        # circle_j2 = plt.Circle((rj2[0], rj2[1]), rad_j2, edgecolor='green', facecolor='none', linewidth=1)
        # axes.add_artist(circle_j2)
        # plt.scatter(pins_x_strand22, pins_y_strand22, color='green', s=30)

        # self.visualize_strand(axes, ri2, rj2, dc2, uc2, n2, n_ci2, n_cj2, r_pi2, r_pj2, up2, pins_x_strand2, pins_y_strand2, rad_i2, rad_j2)
        self.visualize_strand(axes, ri22, rj22, dc22, uc22, n22, n_ci22, n_cj22, r_pi22, r_pj22, up22, pins_x_strand22, pins_y_strand22, rad_i22, rad_j22, 'green')

        # WRAP 1 - 2
        plt.scatter(pins_x_wrap, pins_y_wrap, color='red', s=30)

        # STRAND 3
        self.visualize_strand(axes, ri3, rj3, dc3, uc3, n3, n_ci3, n_cj3, r_pi3, r_pj3, up3, pins_x_strand3, pins_y_strand3, rad_i3, rad_j3, 'magenta')
        
        plt.xlim(-500, 100)
        plt.ylim(-100, 180)

        plt.grid()

        plt.axhline(0, color='black', linewidth=0.1)
        plt.axvline(0, color='black', linewidth=0.1)
        plt.gca().set_aspect('equal', adjustable='box')

        plt.title('2D Chain')
        plt.xlabel('X (Adm Y)')
        plt.ylabel('Y (Adm Z)')

        plt.show()

