# yolo
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# il faut vérifier toutes les valeurs

tau = 10 #@valeur taux compression@ #[-]
D = 0.76 #@valeur alesage@ #[m]
C = 0.8 #@valeur course@ #[m]
L = 0.15 #@valeur longueur bielle@ #[m]
mpiston = 0.5 #@valeur masse piston@ #[kg]
mbielle = 0.3 #@valeur masse bielle@ #[kg]
Q = 2.8 #@valeur chaleur emise par fuel par kg de melange admis@ #[J/kg_inlet gas]

R = C/2
beta = L/R
gamma = 1.3

def Vc():
    return ((np.pi/D^2)/4)*2*R

def W(rpm):
    return rpm*60

def Qtot():
    return Q

def V(theta):
    zp = R*(1 - cos(theta) + beta - sqrt(beta^2 - (sin(theta))^2))
    return Vc()/2 * zp + Vc()/(tau - 1)

def Q(theta, thetaC, deltaThetaC):
    resultat = Qtot()*0.5*(1 - cos(np.pi*((theta - thetaC)/deltaThetaC)))
    return resultat

def dVdTheta():
    return Vc()*0.5*(sin(theta)+(beta^2-(sin(theta))^2)^(-0.5) * cos(theta) * sin(theta))

def dQDTheta(theta, thetaC, deltaThetaC):
    resultat = (Q()*np.pi/(2*deltaThetaC))*sin(np.pi*((theta-thetaC)/deltaThetaC))
    return resultat

def p(rpm, s, theta, thetaC, deltaThetaC):

    def model(p,theta_model):
        dpdTheta = -gamma * (p/V())*dVdTheta() + (gamma - 1) * 1/V() * dQDTheta(theta, thetaC, deltaThetaC)
    return dpdTheta

    # initial condition
    # vérifier que ca vaut s
    y0 = s

    # time points
    #theta_t = np.linspace(0,20)
    theta_t = theta

    # solve ODE
    p = odeint(model,y0,theta_t)

    # plot results
    plt.plot(theta_t,p)
    plt.xlabel('theta')
    plt.ylabel('p(theta)')
    plt.show()


"""
INPUT :

rpm : la vitesse du moteur
s : le taux de suralimentation  (le taux de suralimentation multiplie la pression atmosphérique, supposée égale à 1 bar, pour obtenir la pression d'admission)
theta : l'angle de rotation (theta, en degré, sous forme d'un vecteur de -180 degrés à 180 degrés)
thethaC : l'angle d'allumage (thetaC en degré d'angle de villebrequin avant point mort haut - thetaC = 30 veut dire 30 degrés avant PMH),
deltaThetaC : la durée de la combustion (deltaThetaC en degré d'angle de villebrequin).

OUTPUT :

V_output, en [m3] : l'évolution du volume du cylindre en fonction de l'angle de rotation (theta)
Q_output, en [J] : l'évolution le l'apport de chaleur dans le cylindre en fonction de l'angle de rotation (theta)
F_pied_output et F_tete_output, en [N] : l'évolution des differentes forces s'appliquant sur la bielle en fonction de l'angle de rotation (theta)
p_output, en [Pa] : l'évolution de la pression dans le cylindre en fonction de l'angle de rotation (theta)
t en [m] : l'épaisseur critique, (voir schéma de l'énoncé)
"""
def myfunc(rpm, s, theta, thetaC, deltaThetaC):
    #VOTRE CODE

    #V_output = Vc()/2 * (1 - cos(theta) + beta - sqrt(beta^2 - (sin(theta))^2) ) + Vc()/(tau - 1)

    # j'ai mis ca plus haut
    #zp = R*(1 - cos(theta) + beta - sqrt(beta^2 - (sin(theta))^2))
    #V_output = Vc()/2 * zp + Vc()/(tau - 1)

    V_output = V()

    # Fp
    p_theta = p(rpm, s, theta, thetaC, deltaThetaC)
    fraction = ()(np.pi*D^2)/4)*p_theta
    RW2Cos = R*(W(rpm))^2*cos(theta)

    Fp = fraction - mpiston*RW2Cos
    Ft = -fraction+(mpiston+mbielle)*RW2Cos


    return (V_output, Q_output, F_pied_output, F_tete_output, p_output, t)  ;
