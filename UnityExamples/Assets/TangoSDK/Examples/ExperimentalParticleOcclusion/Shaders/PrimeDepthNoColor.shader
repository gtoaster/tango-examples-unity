Shader "Custom/PrimeDepthNoColor"
{
    Properties
    {
    }
    SubShader
    {
        Tags { "Queue"="Geometry-10" }
        Pass
        {            
            ColorMask 0
            ZWrite On
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = mul(UNITY_MATRIX_MVP, v.vertex);
                return o;
            }
            
            half4 frag (v2f i) : SV_Target
            {
                return half4(0,0,0,0);
            }
            ENDCG
        }
    }
}
