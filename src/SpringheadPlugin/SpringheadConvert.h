
inline Spr::Vec3d ToSpr(const Vector3& v){
	return Spr::Vec3d(v.x(), v.y(), v.z());
}
inline Spr::Matrix3d ToSpr(const Matrix3& m){
	Spr::Matrix3d y;
	for(int i = 0; i < 3; i++)for(int j = 0; j < 3; j++)
		y[i][j] = m(i,j);
	return y;
}

inline Vector3 FromSpr(const Spr::Vec3d& v){
	return Vector3(v.x, v.y, v.z);
}
inline Matrix3 FromSpr(const Spr::Matrix3d& m){
	Matrix3 y;
	for(int i = 0; i < 3; i++)for(int j = 0; j < 3; j++)
		y(i,j) = m[i][j];
	return y;	
}
