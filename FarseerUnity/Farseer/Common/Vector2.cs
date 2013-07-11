using System;
using Vector2 = UnityEngine.Vector2;

namespace Microsoft.Xna.Framework
{
	public static class VectorMath
	{
		private static Vector2 zeroVector = new Vector2( 0.0f, 0.0f );

		public static Vector2 Add( Vector2 value1, Vector2 value2 )
		{
			value1.x += value2.x;
			value1.y += value2.y;
			return value1;
		}

		public static void Add( ref Vector2 value1, ref Vector2 value2, out Vector2 result )
		{
			result.x = value1.x + value2.x;
			result.y = value1.y + value2.y;
		}

		public static Vector2 Subtract( Vector2 value1, Vector2 value2 )
		{
			value1.x -= value2.x;
			value1.y -= value2.y;
			return value1;
		}

		public static void Subtract( ref Vector2 value1, ref Vector2 value2, out Vector2 result )
		{
			result.x = value1.x - value2.x;
			result.y = value1.y - value2.y;
		}

		public static Vector2 Divide( Vector2 value1, Vector2 value2 )
		{
			value1.x /= value2.x;
			value1.y /= value2.y;
			return value1;
		}

		public static void Divide( ref Vector2 value1, ref Vector2 value2, out Vector2 result )
		{
			result.x = value1.x / value2.x;
			result.y = value1.y / value2.y;
		}

		public static Vector2 Divide( Vector2 value1, float divider )
		{
			float factor = 1 / divider;
			value1.x *= factor;
			value1.y *= factor;
			return value1;
		}

		public static void Divide( ref Vector2 value1, float divider, out Vector2 result )
		{
			float factor = 1 / divider;
			result.x = value1.x * factor;
			result.y = value1.y * factor;
		}

		public static Vector2 Multiply( Vector2 value1, Vector2 value2 )
		{
			value1.x *= value2.x;
			value1.y *= value2.y;
			return value1;
		}

		public static Vector2 Multiply( Vector2 value1, float scaleFactor )
		{
			value1.x *= scaleFactor;
			value1.y *= scaleFactor;
			return value1;
		}

		public static void Multiply( ref Vector2 value1, float scaleFactor, out Vector2 result )
		{
			result.x = value1.x * scaleFactor;
			result.y = value1.y * scaleFactor;
		}

		public static void Multiply( ref Vector2 value1, ref Vector2 value2, out Vector2 result )
		{
			result.x = value1.x * value2.x;
			result.y = value1.y * value2.y;
		}

		public static Vector2 Negate( Vector2 value )
		{
			value.x = -value.x;
			value.y = -value.y;
			return value;
		}

		public static void Negate( ref Vector2 value, out Vector2 result )
		{
			result.x = -value.x;
			result.y = -value.y;
		}

		public static float Dot( Vector2 value1, Vector2 value2 )
		{
			return value1.x * value2.x + value1.y * value2.y;
		}

		public static void Dot( ref Vector2 value1, ref Vector2 value2, out float result )
		{
			result = value1.x * value2.x + value1.y * value2.y;
		}

		public static Vector2 Max( Vector2 value1, Vector2 value2 )
		{
			return new Vector2(
				MathHelper.Max( value1.x, value2.x ),
				MathHelper.Max( value1.y, value2.y ) );
		}

		public static void Max( ref Vector2 value1, ref Vector2 value2, out Vector2 result )
		{
			result = new Vector2(
				MathHelper.Max( value1.x, value2.x ),
				MathHelper.Max( value1.y, value2.y ) );
		}

		public static Vector2 Min( Vector2 value1, Vector2 value2 )
		{
			return new Vector2(
				MathHelper.Min( value1.x, value2.x ),
				MathHelper.Min( value1.y, value2.y ) );
		}

		public static void Min( ref Vector2 value1, ref Vector2 value2, out Vector2 result )
		{
			result = new Vector2(
				MathHelper.Min( value1.x, value2.x ),
				MathHelper.Min( value1.y, value2.y ) );
		}

		public static Vector2 Transform( Vector2 position, Matrix matrix )
		{
			Transform( ref position, ref matrix, out position );
			return position;
		}

		public static void Transform( ref Vector2 position, ref Matrix matrix, out Vector2 result )
		{
			result = new Vector2( (position.x*matrix.M11) + (position.y*matrix.M21) + matrix.M41,
			                     (position.x * matrix.M12) + (position.y * matrix.M22) + matrix.M42 );
		}

		public static void Transform( Vector2[] sourceArray, ref Matrix matrix, Vector2[] destinationArray )
		{
			throw new NotImplementedException(); // TODO: Implement.
		}

		public static void Transform( Vector2[] sourceArray, int sourceIndex, ref Matrix matrix,
		                             Vector2[] destinationArray, int destinationIndex, int length )
		{
			throw new NotImplementedException(); // TODO: Implement.
		}

		public static Vector2 CatmullRom( Vector2 value1, Vector2 value2, Vector2 value3, Vector2 value4, float amount )
		{
			return new Vector2(
				MathHelper.CatmullRom( value1.x, value2.x, value3.x, value4.x, amount ),
				MathHelper.CatmullRom( value1.y, value2.y, value3.y, value4.y, amount ) );
		}

		public static void CatmullRom( ref Vector2 value1, ref Vector2 value2, ref Vector2 value3, ref Vector2 value4,
		                              float amount, out Vector2 result )
		{
			result = new Vector2(
				MathHelper.CatmullRom( value1.x, value2.x, value3.x, value4.x, amount ),
				MathHelper.CatmullRom( value1.y, value2.y, value3.y, value4.y, amount ) );
		}

		public static Vector2 Normalize( Vector2 value )
		{
			Normalize( ref value, out value );
			return value;
		}

		public static void Normalize( ref Vector2 value, out Vector2 result )
		{
			float factor;
			DistanceSquared( ref value, ref zeroVector, out factor );
			factor = 1f / (float) Math.Sqrt( factor );
			result.x = value.x * factor;
			result.y = value.y * factor;
		}

		public static float Distance( Vector2 value1, Vector2 value2 )
		{
			float result;
			DistanceSquared( ref value1, ref value2, out result );
			return (float) Math.Sqrt( result );
		}

		public static void Distance( ref Vector2 value1, ref Vector2 value2, out float result )
		{
			DistanceSquared( ref value1, ref value2, out result );
			result = (float) Math.Sqrt( result );
		}

		public static float DistanceSquared( Vector2 value1, Vector2 value2 )
		{
			float result;
			DistanceSquared( ref value1, ref value2, out result );
			return result;
		}

		public static void DistanceSquared( ref Vector2 value1, ref Vector2 value2, out float result )
		{
			result = (value1.x - value2.x) * (value1.x - value2.x) + (value1.y - value2.y) * (value1.y - value2.y);
		}
	}
}
