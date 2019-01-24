package guara;

//public class guaraCinematica {
//
//	public guaraCinematica() {
//		// TODO Auto-generated constructor stub
//	}

//}

//package guara;

public class GuaraKinematics {

	GuaraRobot rob;

	// Variáveis

	double erro = 1.0e-15; // Tolerância para as funções trigonométricas
	double[] X; // vetor de coordenadas do tornozelo o espaço operacional XYZ
	double[] teta; // vetor de coordenadas do tornozelo no espaço de juntas

	double a2,a3,a4; // links da perna
	double x3, y3, z3; // coordenadas cartezianas do tornozelo

	public GuaraKinematics() {
		// TODO Auto-generated constructor stub
		rob = new GuaraRobot();
		a2 = rob.a2();
		a3 = rob.a3();
		a4 = rob.a4();

	}

	// Definição de métodos

	public double[][] DH(double a, double d, double alfa, double teta) {

		/*
		 * Matrizes DH da transformação homogênea entre links Esta listagem é a
		 * implementação em Java do método DH para cálculo da matriz de
		 * transformação entre links da Eq. (5).
		 */
		double cosTeta = (Math.cos(teta) > erro || Math.cos(teta) < -erro) ? Math
				.cos(teta) : 0.0;
		double sinTeta = (Math.sin(teta) > erro || Math.sin(teta) < -erro) ? Math
				.sin(teta) : 0.0;
		double cosAlfa = (Math.cos(alfa) > erro || Math.cos(alfa) < -erro) ? Math
				.cos(alfa) : 0.0;
		double sinAlfa = (Math.sin(alfa) > erro || Math.sin(alfa) < -erro) ? Math
				.sin(alfa) : 0.0;

		double A[][] = new double[4][4];
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				A[i][j] = 0.0;

		A[0][0] = cosTeta;
		A[0][1] = -sinTeta * cosAlfa;
		A[0][2] = sinTeta * sinAlfa;
		A[0][3] = a * cosTeta;

		A[1][0] = sinTeta;
		A[1][1] = cosTeta * cosAlfa;
		A[1][2] = -cosTeta * sinAlfa;
		A[1][3] = a * sinTeta;

		A[2][0] = 0;
		A[2][1] = sinAlfa;
		A[2][2] = cosAlfa;
		A[2][3] = d;

		A[3][0] = 0;
		A[3][1] = 0;
		A[3][2] = 0;
		A[3][3] = 1;

		return A;
	}

	// Cinemática inversa: variáveis da junta 3 (tornozelo) em função da posição
	// [x4,y4,z4] da pata

	public double[] orientacaoCI(double x4, double y4, double z4, double a4,
			double theta1, double theta4) {

		/*
		 * A cinemática inversa de posição é calculada em relação à junta 3
		 * (tornozelo). A partir das coordenadas da pata [x4,y4,z4] obém-se
		 * então as coordenadas da junta 3 [x3,y3,z3] para determinar {teta1,
		 * teta2, teta3, teta4]
		 */

		X = new double[] { 0.0, 0.0, 0.0 };

		X[0] = x4
				- (double) ((a4 * Math.cos(theta4) * Math.sin(theta1) > erro || a4
						* Math.cos(theta4) * Math.sin(theta1) < -erro) ? a4
						* Math.sin(theta4) : 0); // x3

		X[1] = y4
				- (double) ((a4 * Math.cos(theta4) * Math.cos(theta1) > erro || a4
						* Math.cos(theta4) * Math.cos(theta1) < -erro) ? a4
						* Math.sin(theta4) : 0); // y3

		X[2] = z4
				- (double) ((a4 * Math.sin(theta4) > erro || a4
						* Math.sin(theta4) < -erro) ? a4 * Math.sin(theta4) : 0); // z3;
		return X;
	}

	public double[] cinInv(double[] xyz) {

		/*
		 * A cinemática inversa de posição é calculada em relação à junta 3
		 * (tornozelo). A partir das coordenadas da pata [x4,y4,z4] obém-se
		 * então as coordenadas da junta 3 [x3,y3,z3] para determinar {teta1,
		 * teta2, teta3, teta4]
		 */

		double teta[] = {0.0,0.0,0.0,0.0};
		double cosTeta = 0.0, sinTeta = 0.0;

		double x3 = xyz[0];
		double y3 = xyz[1];
		double z3 = xyz[2];

		teta[0] = Math.atan2(y3, x3);
		// cosTeta = (Math.pow(a2, 2) + Math.pow(a3, 2) - Math.pow(y3, 2) - Math
		cosTeta = (Math.pow(x3, 2) + Math.pow(y3, 2) + Math.pow(z3, 2)
				- Math.pow(a2, 2) - Math.pow(a3, 2))
				/ (2 * a2 * a3);
		sinTeta = cosTeta == 0 ? 1.0 : Math.sqrt((1 - Math.pow(cosTeta, 2)));
		teta[2] = Math.atan2(sinTeta, cosTeta);
		double alfa = Math.atan2(a3 * sinTeta, (a2 + a3 * cosTeta));
		double beta = Math.atan2(Math.sqrt(Math.pow(y3, 2) + Math.pow(x3, 2)),
				Math.sqrt(Math.pow(x3, 2) + Math.pow(z3, 2)));
		teta[1] = (Math.abs(z3) > Math.abs(y3)) ? beta + alfa : beta - alfa;
		return teta;
	}

	public double[][] MatrixMultiplication(double[][] m, double[][] n) {
		double c[][] = new double[4][4];
		double sum;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				sum = 0;
				for (int k = 0; k < 4; k++) {
					sum = sum + (m[i][k] * n[k][j]);
					c[i][j] = sum;
				}
			}
		}
		return c;
	}

}

