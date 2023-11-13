        public static double[,] Givens(double[,] A, double[,] L)
        {
            int m = A.GetLength(0), n = A.GetLength(1);
            if (m < n) return GivensLowRank(A, L);
            double Rip = 0, c = 0, s = 0, tmp1 = 0, tmp2 = 0;
            double[,] R = new double[n + 1, n + 1],
                      AL = new double[1, n + 1];
            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                    AL[0, j] = A[i, j];
                AL[0, n] = L[i, 0];
                for (int k = 0; k < n + 1; k++)
                    if (AL[0, k] != 0)
                    {
                        if (R[k, k] != 0)
                        {
                            Rip = Math.Sqrt(AL[0, k] * AL[0, k] + R[k, k] * R[k, k]);
                            c = R[k, k] / Rip;
                            s = AL[0, k] / Rip;
                            for (int p = k; p < n + 1; p++)
                            {
                                tmp1 = R[k, p];
                                tmp2 = AL[0, p];
                                if (tmp1 == 0)
                                    if (tmp2 == 0)
                                        continue;
                                    else
                                    {
                                        R[k, p] = s * tmp2;
                                        AL[0, p] = c * tmp2;
                                    }
                                else
                                    if (tmp2 == 0)
                                    {
                                        R[k, p] = c * tmp1;
                                        AL[0, p] = -s * tmp1;
                                    }
                                    else
                                    {
                                        R[k, p] = c * tmp1 + s * tmp2;
                                        AL[0, p] = -s * tmp1 + c * tmp2;
                                    }
                            }
                        }
                        else
                        {
                            for (int p = k; p < n + 1; p++)
                            {
                                tmp1 = R[k, p];
                                tmp2 = AL[0, p];
                                if (tmp1 == 0)
                                    if (tmp2 == 0)
                                        continue;
                                    else
                                    {
                                        R[k, p] = tmp2;
                                        AL[0, p] = 0;
                                    }
                                else
                                    if (tmp2 == 0)
                                    {
                                        R[k, p] = 0;
                                        AL[0, p] = -tmp1;
                                    }
                                    else
                                    {
                                        R[k, p] = tmp2;
                                        AL[0, p] = -tmp1;
                                    }
                            }
                        }
                    }
            }
            int t = 0;
            for (int i = 0; i < n; i++)
                if (R[i, i] != 0)
                    t++;
            if (t < n)
            {
                double[,] N1 = new double[t, n], W1 = new double[t, 1]; t = 0;
                for (int i = 0; i < n; i++)
                    if (R[i, i] != 0)
                    {
                        for (int j = 0; j < n; j++)
                            N1[t, j] = R[i, j];
                        W1[t++, 0] = R[i, n];
                    }
                return GivensLowRank(N1, W1);
            }

            double[,] x = new double[n + 1, 2];
            double sum = 0,
                   sigma0 = 0, sigma0_pf = 0,
                   sigma = 0, sigma_pf = 0, sum_sigma_pf = 0;
            double maxabslmd = 0, minabslmd = double.MaxValue, kA = 0;
            sigma0 = R[n, n] / Math.Sqrt(m); sigma0_pf = sigma0 * sigma0;
            for (int i = n - 1; i > -1; i--)
            {
                if (maxabslmd < Math.Abs(R[i, i])) maxabslmd = Math.Abs(R[i, i]);
                if (minabslmd > Math.Abs(R[i, i])) minabslmd = Math.Abs(R[i, i]);
                if (R[i, i] == 0) continue;
                sum = 0;
                for (int j = i + 1; j < n; j++)
                    if (R[i, j] != 0 && x[j, 0] != 0)
                        sum += x[j, 0] * R[i, j];
                x[i, 0] = (R[i, n] - sum) / R[i, i];
                sigma_pf = sigma0_pf / R[i, i];
                sum_sigma_pf += sigma_pf;
                x[i, 1] = Math.Sqrt(sigma_pf);
            }
            x[n, 0] = sigma0;
            x[n, 1] = sigma = Math.Sqrt(sum_sigma_pf / n);
            kA = minabslmd == 0 ? double.MaxValue : maxabslmd / minabslmd;

            #region
            if (false)
            {
                System.Drawing.Color colorAL = System.Drawing.Color.Green,
                                     colorR = System.Drawing.Color.Blue;
                System.Drawing.Bitmap bit = new System.Drawing.Bitmap(n + 100 + n + 1, m);
                for (int i = 0; i < m; i++)
                {
                    for (int j = 0; j < n; j++)
                        if (A[i, j] != 0)
                            bit.SetPixel(j, i, colorAL);
                    if (L[i, 0] != 0)
                        bit.SetPixel(n, i, colorAL);
                }
                for (int i = 0; i < n + 1; i++)
                    for (int j = 0; j < n + 1; j++)
                        if (R[i, j] != 0)
                            bit.SetPixel(n + 100 + j, i, colorR);
                bit.Save(@"F:\方程.bmp");
            }
            #endregion

            return x;
        }

        public static double[,] Givens(double[,] A, double[,] L, int r_lower)
        {
            int m = A.GetLength(0), n = A.GetLength(1);
            if (m < n) return GivensLowRank(A, L);
            double Rip = 0, c = 0, s = 0, tmp1 = 0, tmp2 = 0;
            double[,] R = new double[n + 1, n + 1],
                      AL = new double[1, n + 1];
            for (int i = 0; i < m; i++)
            {
                for (int j = 0; j < n; j++)
                    AL[0, j] = A[i, j];
                AL[0, n] = L[i, 0];
                for (int k = 0; k < n + 1; k++)
                    if (AL[0, k] != 0)
                    {
                        if (R[k, k] != 0)
                        {
                            Rip = Math.Sqrt(AL[0, k] * AL[0, k] + R[k, k] * R[k, k]);
                            c = R[k, k] / Rip;
                            s = AL[0, k] / Rip;
                            for (int p = k; p < n + 1; p++)
                            {
                                tmp1 = R[k, p];
                                tmp2 = AL[0, p];
                                if (tmp1 == 0)
                                    if (tmp2 == 0)
                                        continue;
                                    else
                                    {
                                        R[k, p] = s * tmp2;
                                        AL[0, p] = c * tmp2;
                                    }
                                else
                                    if (tmp2 == 0)
                                    {
                                        R[k, p] = c * tmp1;
                                        AL[0, p] = -s * tmp1;
                                    }
                                    else
                                    {
                                        R[k, p] = c * tmp1 + s * tmp2;
                                        AL[0, p] = -s * tmp1 + c * tmp2;
                                    }
                            }
                        }
                        else
                        {
                            for (int p = k; p < n + 1; p++)
                            {
                                tmp1 = R[k, p];
                                tmp2 = AL[0, p];
                                if (tmp1 == 0)
                                    if (tmp2 == 0)
                                        continue;
                                    else
                                    {
                                        R[k, p] = tmp2;
                                        AL[0, p] = 0;
                                    }
                                else
                                    if (tmp2 == 0)
                                    {
                                        R[k, p] = 0;
                                        AL[0, p] = -tmp1;
                                    }
                                    else
                                    {
                                        R[k, p] = tmp2;
                                        AL[0, p] = -tmp1;
                                    }
                            }
                        }
                    }
            }
            int t = n - r_lower;
            if (t < n)
            {
                double[,] N1 = new double[t, n], W1 = new double[t, 1]; t = 0;
                for (int i = 0; i < n; i++)
                    if (R[i, i] != 0 && t < n - r_lower)
                    {
                        for (int j = 0; j < n; j++)
                            N1[t, j] = R[i, j];
                        W1[t++, 0] = R[i, n];
                    }
                return GivensLowRank(N1, W1);
            }

            double[,] x = new double[n + 1, 2];
            double sum = 0,
                   sigma0 = 0, sigma0_pf = 0,
                   sigma = 0, sigma_pf = 0, sum_sigma_pf = 0;
            double maxabslmd = 0, minabslmd = double.MaxValue, kA = 0;
            sigma0 = R[n, n] / Math.Sqrt(m); sigma0_pf = sigma0 * sigma0;
            for (int i = n - 1; i > -1; i--)
            {
                if (maxabslmd < Math.Abs(R[i, i])) maxabslmd = Math.Abs(R[i, i]);
                if (minabslmd > Math.Abs(R[i, i])) minabslmd = Math.Abs(R[i, i]);
                if (R[i, i] == 0) continue;
                sum = 0;
                for (int j = i + 1; j < n; j++)
                    if (R[i, j] != 0 && x[j, 0] != 0)
                        sum += x[j, 0] * R[i, j];
                x[i, 0] = (R[i, n] - sum) / R[i, i];
                sigma_pf = sigma0_pf / R[i, i];
                sum_sigma_pf += sigma_pf;
                x[i, 1] = Math.Sqrt(sigma_pf);
            }
            x[n, 0] = sigma0;
            x[n, 1] = sigma = Math.Sqrt(sum_sigma_pf / n);
            kA = minabslmd == 0 ? double.MaxValue : maxabslmd / minabslmd;

            #region
            if (false)
            {
                System.Drawing.Color colorAL = System.Drawing.Color.Green,
                                     colorR = System.Drawing.Color.Blue;
                System.Drawing.Bitmap bit = new System.Drawing.Bitmap(n + 100 + n + 1, m);
                for (int i = 0; i < m; i++)
                {
                    for (int j = 0; j < n; j++)
                        if (A[i, j] != 0)
                            bit.SetPixel(j, i, colorAL);
                    if (L[i, 0] != 0)
                        bit.SetPixel(n, i, colorAL);
                }
                for (int i = 0; i < n + 1; i++)
                    for (int j = 0; j < n + 1; j++)
                        if (R[i, j] != 0)
                            bit.SetPixel(n + 100 + j, i, colorR);
                bit.Save(@"F:\方程.bmp");
            }
            #endregion

            return x;
        }

        static double[,] GivensLowRank(double[,] N1, double[,] W1)
        {
            int m = N1.GetLength(1), n = N1.GetLength(0);
            double Rip = 0, c = 0, s = 0, tmp1 = 0, tmp2 = 0;
            double[,] N = new double[n, n],
                      R = new double[n + 1, n + 1],
                      AL = new double[1, n + 1];
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    for (int k = 0; k < m; k++)
                        if (N1[i, k] != 0 && N1[j, k] != 0)
                            N[i, j] += N1[i, k] * N1[j, k];

            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                    AL[0, j] = N[i, j];
                AL[0, n] = W1[i, 0];
                for (int k = 0; k < n + 1; k++)
                    if (AL[0, k] != 0)
                    {
                        if (R[k, k] != 0)
                        {
                            Rip = Math.Sqrt(AL[0, k] * AL[0, k] + R[k, k] * R[k, k]);
                            c = R[k, k] / Rip;
                            s = AL[0, k] / Rip;
                            for (int p = k; p < n + 1; p++)
                            {
                                tmp1 = R[k, p];
                                tmp2 = AL[0, p];
                                if (tmp1 == 0)
                                    if (tmp2 == 0)
                                        continue;
                                    else
                                    {
                                        R[k, p] = s * tmp2;
                                        AL[0, p] = c * tmp2;
                                    }
                                else
                                    if (tmp2 == 0)
                                    {
                                        R[k, p] = c * tmp1;
                                        AL[0, p] = -s * tmp1;
                                    }
                                    else
                                    {
                                        R[k, p] = c * tmp1 + s * tmp2;
                                        AL[0, p] = -s * tmp1 + c * tmp2;
                                    }
                            }
                        }
                        else
                        {
                            for (int p = k; p < n + 1; p++)
                            {
                                tmp1 = R[k, p];
                                tmp2 = AL[0, p];
                                if (tmp1 == 0)
                                    if (tmp2 == 0)
                                        continue;
                                    else
                                    {
                                        R[k, p] = tmp2;
                                        AL[0, p] = 0;
                                    }
                                else
                                    if (tmp2 == 0)
                                    {
                                        R[k, p] = 0;
                                        AL[0, p] = -tmp1;
                                    }
                                    else
                                    {
                                        R[k, p] = tmp2;
                                        AL[0, p] = -tmp1;
                                    }
                            }
                        }
                    }
            }

            double[,] y = new double[n, 1];
            double sum = 0;
            for (int i = n - 1; i > -1; i--)
            {
                if (R[i, i] == 0) continue;
                sum = 0;
                for (int j = i + 1; j < n; j++)
                    if (R[i, j] != 0 & y[j, 0] != 0)
                        sum += y[j, 0] * R[i, j];
                y[i, 0] = (R[i, n] - sum) / R[i, i];
            }
            double[,] x = new double[m + 1, 2];
            for (int i = 0; i < m; i++)
                for (int j = 0; j < n; j++)
                    if (N1[j, i] != 0 & y[j, 0] != 0)
                        x[i, 0] += N1[j, i] * y[j, 0];

            #region
            if (false)
            {
                System.Drawing.Color colorAL = System.Drawing.Color.Green,
                                     colorR = System.Drawing.Color.Blue;
                System.Drawing.Bitmap bit = new System.Drawing.Bitmap(n + 100 + n + 1, m);
                for (int i = 0; i < m; i++)
                {
                    for (int j = 0; j < n; j++)
                        if (N1[i, j] != 0)
                            bit.SetPixel(j, i, colorAL);
                    if (W1[i, 0] != 0)
                        bit.SetPixel(n, i, colorAL);
                }
                for (int i = 0; i < n + 1; i++)
                    for (int j = 0; j < n + 1; j++)
                        if (R[i, j] != 0)
                            bit.SetPixel(n + 100 + j, i, colorR);
                bit.Save(@"F:\方程.bmp");
            }
            #endregion

            return x;
        }

