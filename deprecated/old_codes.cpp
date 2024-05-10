
static v3 RayCast(world_t *world, v3 o, v3 d, int depth)
{
    float tolerance = TOLERANCE, minHitDistance = MIN_HIT_DISTANCE;
    v3 rayOrigin, rayDirection;

    v3 radiance = {};
    if (depth>=MAX_BOUNCE_COUNT) return radiance;

    rayOrigin=o;
    rayDirection=d;

    ray_payload_t p;
    float &hitDistance = p.hitDistance;
    unsigned int &hitMatIndex = p.hitMatIndex;
    v3 &nextNormal = p.normal;
    p=RayCastIntersect(world, rayOrigin, rayDirection);
      
    material_t mat = world->materials[hitMatIndex];
    do {
    if (hitMatIndex) {

        float cosTheta,NdotL,NdotV,F0,metalness;
        v3 halfVector,N,L,V,pureBounce,brdfTerm,ks_local,kd_local,r3,tangentX,tangentY;

        cosTheta=(Dot(nextNormal, rayDirection));
        cosTheta=(cosTheta>0.f)?Dot(-1.f*nextNormal, rayDirection):cosTheta;

        F0 = Square((N_AIR-mat.ior)/(N_AIR+mat.ior)); // NOTE: need to change when support refraction again.

        int tapCount = g_sc;
        //float tapContrib = 1.f / float(tapCount + nc_sbcount(world->lights));
        float tapContrib=1.f/tapCount;

        rayOrigin = rayOrigin + hitDistance * rayDirection;
        pureBounce = rayDirection - 2.0f * cosTheta * nextNormal;

        N=Dot(nextNormal,rayDirection)<0.f ? nextNormal:-1.f*nextNormal;
        V=-rayDirection;

        // Via the material mapping (might be UVs or some other function), sample the texture.
        v2 uv = {rayOrigin.x,rayOrigin.y};//for now.

        { // find the metalness.
            if (mat.metalnessIdx==0 || !g_bMetalness) {
                metalness=mat.metalness;
            } else {
                texture_t tex=g_textures[mat.metalnessIdx-1];
                r3 = BespokeSampleTexture( tex, uv );
                metalness=r3.x;
            }
        }

        // find the normal.
        if (g_bNormals && mat.normalIdx!=0){
            texture_t tex=g_textures[mat.normalIdx-1];
            N = BespokeSampleTexture(tex, uv );
            // NOTE: this currently only works for the ground plane, since it's normal happens to be up!
            N = Normalize(2.f*N - V3(1.f,1.f,1.f));
        }

        if (Dot(N, V)<=0.f) break;

        // Define the local tangent space using the normal.
        // I haven't tested this code,could be wrong. but even if it's wrong,output is likely
        // to still be correct. we don't yet support anisotropic materials and all the PDF functions
        // are isotropic.
        if (N.z==0.f){
            tangentY = Normalize(Cross(N, V3(0,1,0)));
            tangentX = Normalize(Cross(tangentY,N));
        }else{
            tangentX = Normalize(Cross(V3(0,1,0), N));
            tangentY = Normalize(Cross(N, tangentX));
        }

#if 0
        // cast the shadow ray(s).
        for (unsigned int i=0;i< nc_sbcount(world->lights);i++) {
            light_t &light=world->lights[i];
            float hitThreshold=FLT_MAX,attenuation=1.f;
            switch(light.kind){
                case LIGHT_KIND_DIRECTIONAL:
                {
                    L=-1.f*light.direction;
                } break;
                case LIGHT_KIND_POINT:
                {
                    L=light.position-rayOrigin;
                    hitThreshold=Magnitude(L);
                    L=(1.f/hitThreshold)*L;//normalize.
                    attenuation=powf(2.7f,-0.1f*hitThreshold);
                } break;
                case LIGHT_KIND_TRIANGLE:
                break;
            }

            halfVector =(1.f/Magnitude(L+V)) * (L+V);
            cosTheta=Dot(halfVector,L);

            if ((NdotL=Dot(N, L))>0.f && (Dot(halfVector,V)>0.f) && cosTheta>0.f && attenuation>0.f) {
                assert(cosTheta>=0.f);
                ks_local = SchlickMetal(F0,cosTheta,metalness,mat.metalColor);
                kd_local=V3(1.f,1.f,1.f)-ks_local;
                for (int i = 0;i < 3;i++) assert(ks_local.E[i] >= 0.f && ks_local.E[i] <= 1.f);
                kd_local = Lerp(kd_local, V3(0,0,0), metalness); // metal surfaces have a very high absorption!
                brdfTerm = Hadamard(kd_local, brdf_diff(mat,rayOrigin))+
                    Hadamard(ks_local, brdf_specular(mat,rayOrigin, N, L, V, halfVector ));

                auto payload=RayCastIntersect(world,rayOrigin,L);
                if (payload.hitMatIndex==0 || (payload.hitDistance>hitThreshold&&payload.hitDistance>minHitDistance) ) {
                    //radiance += tapContrib * NdotL * Hadamard( attenuation*light.radiance, brdfTerm );
                    radiance += NdotL * Hadamard( attenuation*light.radiance, brdfTerm );//correct coeff for importance sampling, i.e. inclusion of 1/p(x) term.
                }
            }
        } // END cast the shadow rays.
#endif

        // use monte carlo estimator for the specular lobe.
        for (int i=0;i<tapCount;i++)
        {
            float theta,phi,x,y,z,randX,px;

            if (EffectivelySmooth(mat.roughness)) {
                L=pureBounce;
                px=100.f; //the probability distribution for a perfectly smooth surface is a dirac delta.
                //i++
                NdotL=Dot(N, L);
            } else {
                
                v3 rDir=RandomCosineDirectionHemisphere();
                v3 lobeBounce = Normalize(rDir.x*tangentX+rDir.y*tangentY+rDir.z*N);
                L = lobeBounce;
                px=(NdotL=Dot(N, L))/PI;
            }            

            halfVector =(1.f/Magnitude(L+V)) * (L+V);
            cosTheta=Dot(halfVector,L);

            if ((NdotL)>0.f&&(Dot(halfVector,V)>0.f)&&cosTheta>0.f)//incoming light is in hemisphere.
            {
                //i++;
                ks_local = SchlickMetal(F0,cosTheta,metalness,mat.metalColor);
                kd_local=V3(1.f,1.f,1.f)-ks_local;
                for(int j=0;j<3;j++) assert(ks_local.E[j] >= 0.f && ks_local.E[j] <= 1.f);
                kd_local = Lerp(kd_local, V3(0,0,0), metalness); // metal surfaces have a very high absorption!
                brdfTerm = Hadamard(ks_local, brdf_specular(mat,rayOrigin, N, L, V, halfVector ) );

                radiance += tapContrib * 
                    (1.f/px) * NdotL * Hadamard(RayCast(world,rayOrigin,L,depth+1), brdfTerm);
            }
        }

        // use monte carlo estimator for the diffuse lobe.
        for (int i=0;i<tapCount;i++)
        {
            float px;

            v3 rDir=RandomCosineDirectionHemisphere();
            v3 lobeBounce = Normalize(rDir.x*tangentX+rDir.y*tangentY+rDir.z*N);
            L = lobeBounce;
            px = 1.f/PI;

            halfVector =(1.f/Magnitude(L+V)) * (L+V);
            cosTheta=Dot(halfVector,L);

            if ((NdotL=Dot(N, L))>0.f)//incoming light is in hemisphere.
            {
                assert(cosTheta>0.f);

                ks_local = SchlickMetal(F0,NdotL,metalness,mat.metalColor);
                kd_local=V3(1.f,1.f,1.f)-ks_local;
                for(int j=0;j<3;j++) assert(ks_local.E[j] >= 0.f && ks_local.E[j] <= 1.f);
                kd_local = Lerp(kd_local, V3(0,0,0), metalness); // metal surfaces have a very high absorption!
                brdfTerm = Hadamard(kd_local, brdf_diff(mat,rayOrigin));

                // NOTE: since we sample by cos(theta), the NdotL term goes away by the 1/p(x) term.
                radiance += (1.f/px) * tapContrib * Hadamard(RayCast(world,rayOrigin,L,depth+1), brdfTerm);
            }
        } // END spawning the bounce rays.

        // How to think about the refraction ray?
        // it doesn't really have anything to do with the BRDF.
        // it's simply where we continue along the same ray path,
        // but there's a bend in the path along the way (due to the interface).
        // and we treat the geometry as "participating media", where
        // there is some absorption when transporting through a translucent
        // material.
        
        // coming up through a transparent surface
        // with the reflection ray calculated as 'Iout' above. The blend
        // should be 'opacity' of the reflected ray and '1-opacity' of the
        // refracted ray.

        // disable refraction for now.
#if 0
        if (mat.alpha < 1.0f) { // not completely opaque.

            v3 refractionDirection;
            if (FindRefractionDirection(rayDirection, nextNormal, mat.ior, refractionDirection))
                radiance += kd*(1.f-mat.alpha) * RayCast(world, rayOrigin, refractionDirection, depth);
        }
#endif

    } // END IF.
    } while(false);

    radiance += mat.emitColor;

    return radiance;
}


// SAMPLING THAT IS NOT STATIFIED.
// NOTE: this path is now not tested,after the camera refactor.
contrib = 1.0f / (float)g_pp;
for (unsigned int rayIndex = 0; rayIndex < g_pp; rayIndex++) {
    float offX = filmX + (RandomBilateral() * g_camera.halfFilmPixelW);
    float offY = filmY + (RandomBilateral() * g_camera.halfFilmPixelH);
    filmP = g_camera.filmCenter + 
        (offX * g_camera.halfFilmWidth * g_camera.axisX) +
        (offY * g_camera.halfFilmHeight * g_camera.axisY);
    rayDirection = Normalize(filmP - g_camera.pos);
    color = color + contrib * RayCastFast(&g_world, rayOrigin, rayDirection,0);
}

v3 RandomDirectionHemisphere(){
    float theta,phi,x,y,z;
    theta = RandomUnilateral()*2.f*PI;
    phi   = acos(z=(1.f-RandomUnilateral()/**0.5f*2.f*/));
    x = sin(phi) * cos(theta);
    y = sin(phi) * sin(theta);
    return V3(x,y,z);
}

float Schlick(float F0, float cosTheta) {
    return F0+(1.f-F0)*powf(1.f-cosTheta,5.f);
}

float MaskingShadowing(v3 normal, v3 L, v3 V, v3 H, float roughness){
    float a     = roughness;
    float Lambda(v3 N, v3 s,float a);
    //if (Dot(H,V)<=0.f) return 0.f;
    float denom = 1.f+Lambda(normal,V,a)+Lambda(normal,L,a);
    if (denom==0.f)return 1.f;
    return 1.f/(denom);
}
float Lambda(v3 N, v3 s,float a){
    return 0.5f*((a/Dot(N,s))-1.f);
}