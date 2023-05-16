let wasm;

const heap = new Array(128).fill(undefined);

heap.push(undefined, null, true, false);

function getObject(idx) { return heap[idx]; }

let heap_next = heap.length;

function dropObject(idx) {
    if (idx < 132) return;
    heap[idx] = heap_next;
    heap_next = idx;
}

function takeObject(idx) {
    const ret = getObject(idx);
    dropObject(idx);
    return ret;
}

let WASM_VECTOR_LEN = 0;

let cachedUint8Memory0 = null;

function getUint8Memory0() {
    if (cachedUint8Memory0 === null || cachedUint8Memory0.byteLength === 0) {
        cachedUint8Memory0 = new Uint8Array(wasm.memory.buffer);
    }
    return cachedUint8Memory0;
}

const cachedTextEncoder = (typeof TextEncoder !== 'undefined' ? new TextEncoder('utf-8') : { encode: () => { throw Error('TextEncoder not available') } } );

const encodeString = (typeof cachedTextEncoder.encodeInto === 'function'
    ? function (arg, view) {
    return cachedTextEncoder.encodeInto(arg, view);
}
    : function (arg, view) {
    const buf = cachedTextEncoder.encode(arg);
    view.set(buf);
    return {
        read: arg.length,
        written: buf.length
    };
});

function passStringToWasm0(arg, malloc, realloc) {

    if (realloc === undefined) {
        const buf = cachedTextEncoder.encode(arg);
        const ptr = malloc(buf.length) >>> 0;
        getUint8Memory0().subarray(ptr, ptr + buf.length).set(buf);
        WASM_VECTOR_LEN = buf.length;
        return ptr;
    }

    let len = arg.length;
    let ptr = malloc(len) >>> 0;

    const mem = getUint8Memory0();

    let offset = 0;

    for (; offset < len; offset++) {
        const code = arg.charCodeAt(offset);
        if (code > 0x7F) break;
        mem[ptr + offset] = code;
    }

    if (offset !== len) {
        if (offset !== 0) {
            arg = arg.slice(offset);
        }
        ptr = realloc(ptr, len, len = offset + arg.length * 3) >>> 0;
        const view = getUint8Memory0().subarray(ptr + offset, ptr + len);
        const ret = encodeString(arg, view);

        offset += ret.written;
    }

    WASM_VECTOR_LEN = offset;
    return ptr;
}

function isLikeNone(x) {
    return x === undefined || x === null;
}

let cachedInt32Memory0 = null;

function getInt32Memory0() {
    if (cachedInt32Memory0 === null || cachedInt32Memory0.byteLength === 0) {
        cachedInt32Memory0 = new Int32Array(wasm.memory.buffer);
    }
    return cachedInt32Memory0;
}

let cachedFloat64Memory0 = null;

function getFloat64Memory0() {
    if (cachedFloat64Memory0 === null || cachedFloat64Memory0.byteLength === 0) {
        cachedFloat64Memory0 = new Float64Array(wasm.memory.buffer);
    }
    return cachedFloat64Memory0;
}

function addHeapObject(obj) {
    if (heap_next === heap.length) heap.push(heap.length + 1);
    const idx = heap_next;
    heap_next = heap[idx];

    heap[idx] = obj;
    return idx;
}

const cachedTextDecoder = (typeof TextDecoder !== 'undefined' ? new TextDecoder('utf-8', { ignoreBOM: true, fatal: true }) : { decode: () => { throw Error('TextDecoder not available') } } );

if (typeof TextDecoder !== 'undefined') { cachedTextDecoder.decode(); };

function getStringFromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return cachedTextDecoder.decode(getUint8Memory0().subarray(ptr, ptr + len));
}

function debugString(val) {
    // primitive types
    const type = typeof val;
    if (type == 'number' || type == 'boolean' || val == null) {
        return  `${val}`;
    }
    if (type == 'string') {
        return `"${val}"`;
    }
    if (type == 'symbol') {
        const description = val.description;
        if (description == null) {
            return 'Symbol';
        } else {
            return `Symbol(${description})`;
        }
    }
    if (type == 'function') {
        const name = val.name;
        if (typeof name == 'string' && name.length > 0) {
            return `Function(${name})`;
        } else {
            return 'Function';
        }
    }
    // objects
    if (Array.isArray(val)) {
        const length = val.length;
        let debug = '[';
        if (length > 0) {
            debug += debugString(val[0]);
        }
        for(let i = 1; i < length; i++) {
            debug += ', ' + debugString(val[i]);
        }
        debug += ']';
        return debug;
    }
    // Test for built-in
    const builtInMatches = /\[object ([^\]]+)\]/.exec(toString.call(val));
    let className;
    if (builtInMatches.length > 1) {
        className = builtInMatches[1];
    } else {
        // Failed to match the standard '[object ClassName]'
        return toString.call(val);
    }
    if (className == 'Object') {
        // we're a user defined class or Object
        // JSON.stringify avoids problems with cycles, and is generally much
        // easier than looping through ownProperties of `val`.
        try {
            return 'Object(' + JSON.stringify(val) + ')';
        } catch (_) {
            return 'Object';
        }
    }
    // errors
    if (val instanceof Error) {
        return `${val.name}: ${val.message}\n${val.stack}`;
    }
    // TODO we could test for more things here, like `Set`s and `Map`s.
    return className;
}

function makeMutClosure(arg0, arg1, dtor, f) {
    const state = { a: arg0, b: arg1, cnt: 1, dtor };
    const real = (...args) => {
        // First up with a closure we increment the internal reference
        // count. This ensures that the Rust closure environment won't
        // be deallocated while we're invoking it.
        state.cnt++;
        const a = state.a;
        state.a = 0;
        try {
            return f(a, state.b, ...args);
        } finally {
            if (--state.cnt === 0) {
                wasm.__wbindgen_export_2.get(state.dtor)(a, state.b);

            } else {
                state.a = a;
            }
        }
    };
    real.original = state;

    return real;
}
function __wbg_adapter_26(arg0, arg1, arg2) {
    wasm.wasm_bindgen__convert__closures__invoke1_mut__h31d1fe5e1be97fca(arg0, arg1, addHeapObject(arg2));
}

function __wbg_adapter_43(arg0, arg1) {
    wasm.wasm_bindgen__convert__closures__invoke0_mut__h28c818ae5169b819(arg0, arg1);
}

/**
*/
export function run_js() {
    wasm.run_js();
}

function handleError(f, args) {
    try {
        return f.apply(this, args);
    } catch (e) {
        wasm.__wbindgen_exn_store(addHeapObject(e));
    }
}

let cachedFloat32Memory0 = null;

function getFloat32Memory0() {
    if (cachedFloat32Memory0 === null || cachedFloat32Memory0.byteLength === 0) {
        cachedFloat32Memory0 = new Float32Array(wasm.memory.buffer);
    }
    return cachedFloat32Memory0;
}

function getArrayF32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getFloat32Memory0().subarray(ptr / 4, ptr / 4 + len);
}

function getArrayI32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getInt32Memory0().subarray(ptr / 4, ptr / 4 + len);
}

let cachedUint32Memory0 = null;

function getUint32Memory0() {
    if (cachedUint32Memory0 === null || cachedUint32Memory0.byteLength === 0) {
        cachedUint32Memory0 = new Uint32Array(wasm.memory.buffer);
    }
    return cachedUint32Memory0;
}

function getArrayU32FromWasm0(ptr, len) {
    ptr = ptr >>> 0;
    return getUint32Memory0().subarray(ptr / 4, ptr / 4 + len);
}

async function __wbg_load(module, imports) {
    if (typeof Response === 'function' && module instanceof Response) {
        if (typeof WebAssembly.instantiateStreaming === 'function') {
            try {
                return await WebAssembly.instantiateStreaming(module, imports);

            } catch (e) {
                if (module.headers.get('Content-Type') != 'application/wasm') {
                    console.warn("`WebAssembly.instantiateStreaming` failed because your server does not serve wasm with `application/wasm` MIME type. Falling back to `WebAssembly.instantiate` which is slower. Original error:\n", e);

                } else {
                    throw e;
                }
            }
        }

        const bytes = await module.arrayBuffer();
        return await WebAssembly.instantiate(bytes, imports);

    } else {
        const instance = await WebAssembly.instantiate(module, imports);

        if (instance instanceof WebAssembly.Instance) {
            return { instance, module };

        } else {
            return instance;
        }
    }
}

function __wbg_get_imports() {
    const imports = {};
    imports.wbg = {};
    imports.wbg.__wbindgen_object_drop_ref = function(arg0) {
        takeObject(arg0);
    };
    imports.wbg.__wbindgen_string_get = function(arg0, arg1) {
        const obj = getObject(arg1);
        const ret = typeof(obj) === 'string' ? obj : undefined;
        var ptr1 = isLikeNone(ret) ? 0 : passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        var len1 = WASM_VECTOR_LEN;
        getInt32Memory0()[arg0 / 4 + 1] = len1;
        getInt32Memory0()[arg0 / 4 + 0] = ptr1;
    };
    imports.wbg.__wbg_createFramebuffer_fa9f0e78e44a757b = function(arg0) {
        const ret = getObject(arg0).createFramebuffer();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_createFramebuffer_71938241c433e68a = function(arg0) {
        const ret = getObject(arg0).createFramebuffer();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_createShader_9872e1b3afd8d089 = function(arg0, arg1) {
        const ret = getObject(arg0).createShader(arg1 >>> 0);
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_createShader_5ce8581c05641306 = function(arg0, arg1) {
        const ret = getObject(arg0).createShader(arg1 >>> 0);
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_createTexture_5199a262e1135922 = function(arg0) {
        const ret = getObject(arg0).createTexture();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_createTexture_0085c2b16877a463 = function(arg0) {
        const ret = getObject(arg0).createTexture();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_deleteShader_dff44e945e31d0bc = function(arg0, arg1) {
        getObject(arg0).deleteShader(getObject(arg1));
    };
    imports.wbg.__wbg_deleteShader_d2b618b4b0adc1d6 = function(arg0, arg1) {
        getObject(arg0).deleteShader(getObject(arg1));
    };
    imports.wbg.__wbg_shaderSource_ee6736760085f6d5 = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).shaderSource(getObject(arg1), getStringFromWasm0(arg2, arg3));
    };
    imports.wbg.__wbg_shaderSource_ada4481e7f64b59b = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).shaderSource(getObject(arg1), getStringFromWasm0(arg2, arg3));
    };
    imports.wbg.__wbg_compileShader_05a9708569d83fce = function(arg0, arg1) {
        getObject(arg0).compileShader(getObject(arg1));
    };
    imports.wbg.__wbg_compileShader_b5c1669afdfaec13 = function(arg0, arg1) {
        getObject(arg0).compileShader(getObject(arg1));
    };
    imports.wbg.__wbindgen_boolean_get = function(arg0) {
        const v = getObject(arg0);
        const ret = typeof(v) === 'boolean' ? (v ? 1 : 0) : 2;
        return ret;
    };
    imports.wbg.__wbg_createProgram_bd425772d6f68f5b = function(arg0) {
        const ret = getObject(arg0).createProgram();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_createProgram_cba5d9c3dccfecab = function(arg0) {
        const ret = getObject(arg0).createProgram();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_deleteProgram_6c6fcb7fcee7d0ce = function(arg0, arg1) {
        getObject(arg0).deleteProgram(getObject(arg1));
    };
    imports.wbg.__wbg_deleteProgram_b47fbad8d56ac79a = function(arg0, arg1) {
        getObject(arg0).deleteProgram(getObject(arg1));
    };
    imports.wbg.__wbg_attachShader_5ae5c90303b18698 = function(arg0, arg1, arg2) {
        getObject(arg0).attachShader(getObject(arg1), getObject(arg2));
    };
    imports.wbg.__wbg_attachShader_508b395353cfea3c = function(arg0, arg1, arg2) {
        getObject(arg0).attachShader(getObject(arg1), getObject(arg2));
    };
    imports.wbg.__wbg_linkProgram_92812e7e1d6f1964 = function(arg0, arg1) {
        getObject(arg0).linkProgram(getObject(arg1));
    };
    imports.wbg.__wbg_linkProgram_59a69c0279983602 = function(arg0, arg1) {
        getObject(arg0).linkProgram(getObject(arg1));
    };
    imports.wbg.__wbindgen_number_get = function(arg0, arg1) {
        const obj = getObject(arg1);
        const ret = typeof(obj) === 'number' ? obj : undefined;
        getFloat64Memory0()[arg0 / 8 + 1] = isLikeNone(ret) ? 0 : ret;
        getInt32Memory0()[arg0 / 4 + 0] = !isLikeNone(ret);
    };
    imports.wbg.__wbg_useProgram_79cf53ff131a1570 = function(arg0, arg1) {
        getObject(arg0).useProgram(getObject(arg1));
    };
    imports.wbg.__wbg_useProgram_cbbec0eb937f0880 = function(arg0, arg1) {
        getObject(arg0).useProgram(getObject(arg1));
    };
    imports.wbg.__wbg_createBuffer_ce15a68e1252d581 = function(arg0) {
        const ret = getObject(arg0).createBuffer();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_createBuffer_ad18673ecf93364a = function(arg0) {
        const ret = getObject(arg0).createBuffer();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_bindBuffer_e0dced5f2ece74e8 = function(arg0, arg1, arg2) {
        getObject(arg0).bindBuffer(arg1 >>> 0, getObject(arg2));
    };
    imports.wbg.__wbg_bindBuffer_1c07a1c1b4badd6b = function(arg0, arg1, arg2) {
        getObject(arg0).bindBuffer(arg1 >>> 0, getObject(arg2));
    };
    imports.wbg.__wbg_bindFramebuffer_50e959deb5431956 = function(arg0, arg1, arg2) {
        getObject(arg0).bindFramebuffer(arg1 >>> 0, getObject(arg2));
    };
    imports.wbg.__wbg_bindFramebuffer_aa4326d94595717d = function(arg0, arg1, arg2) {
        getObject(arg0).bindFramebuffer(arg1 >>> 0, getObject(arg2));
    };
    imports.wbg.__wbg_bindRenderbuffer_38ae00f431742547 = function(arg0, arg1, arg2) {
        getObject(arg0).bindRenderbuffer(arg1 >>> 0, getObject(arg2));
    };
    imports.wbg.__wbg_bindRenderbuffer_8448dec7e451d7fe = function(arg0, arg1, arg2) {
        getObject(arg0).bindRenderbuffer(arg1 >>> 0, getObject(arg2));
    };
    imports.wbg.__wbg_pixelStorei_0c9f1b244b01bb66 = function(arg0, arg1, arg2) {
        getObject(arg0).pixelStorei(arg1 >>> 0, arg2);
    };
    imports.wbg.__wbg_pixelStorei_a7a1a09a34a88ba4 = function(arg0, arg1, arg2) {
        getObject(arg0).pixelStorei(arg1 >>> 0, arg2);
    };
    imports.wbg.__wbg_bufferSubData_e6bdcff2e33b80b0 = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).bufferSubData(arg1 >>> 0, arg2, getObject(arg3));
    };
    imports.wbg.__wbg_bufferSubData_a3d79ed0b605b30a = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).bufferSubData(arg1 >>> 0, arg2, getObject(arg3));
    };
    imports.wbg.__wbg_getBufferSubData_ca134d28f22653bf = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).getBufferSubData(arg1 >>> 0, arg2, getObject(arg3));
    };
    imports.wbg.__wbg_clearBufferiv_6d87adc14fc5380a = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).clearBufferiv(arg1 >>> 0, arg2, getArrayI32FromWasm0(arg3, arg4));
    };
    imports.wbg.__wbg_clearBufferfv_2319e28e0f6bcbe9 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).clearBufferfv(arg1 >>> 0, arg2, getArrayF32FromWasm0(arg3, arg4));
    };
    imports.wbg.__wbg_copyBufferSubData_b5d07472fe72da45 = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).copyBufferSubData(arg1 >>> 0, arg2 >>> 0, arg3, arg4, arg5);
    };
    imports.wbg.__wbg_deleteBuffer_43f1c1c4d32100d1 = function(arg0, arg1) {
        getObject(arg0).deleteBuffer(getObject(arg1));
    };
    imports.wbg.__wbg_deleteBuffer_b622dabf349015ab = function(arg0, arg1) {
        getObject(arg0).deleteBuffer(getObject(arg1));
    };
    imports.wbg.__wbg_deleteFramebuffer_2692f9309b02472d = function(arg0, arg1) {
        getObject(arg0).deleteFramebuffer(getObject(arg1));
    };
    imports.wbg.__wbg_deleteFramebuffer_bce5278d23d4fed5 = function(arg0, arg1) {
        getObject(arg0).deleteFramebuffer(getObject(arg1));
    };
    imports.wbg.__wbg_deleteSync_dd2cf9bffbef1f79 = function(arg0, arg1) {
        getObject(arg0).deleteSync(getObject(arg1));
    };
    imports.wbg.__wbg_deleteTexture_3bff2c6d387bdb53 = function(arg0, arg1) {
        getObject(arg0).deleteTexture(getObject(arg1));
    };
    imports.wbg.__wbg_deleteTexture_cc293d409b2ac2da = function(arg0, arg1) {
        getObject(arg0).deleteTexture(getObject(arg1));
    };
    imports.wbg.__wbg_disable_0ea446922c277ac9 = function(arg0, arg1) {
        getObject(arg0).disable(arg1 >>> 0);
    };
    imports.wbg.__wbg_disable_fc5f4d048c61339f = function(arg0, arg1) {
        getObject(arg0).disable(arg1 >>> 0);
    };
    imports.wbg.__wbg_drawArrays_c5191d7787a88a21 = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).drawArrays(arg1 >>> 0, arg2, arg3);
    };
    imports.wbg.__wbg_drawArrays_cde0f84c08ae294f = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).drawArrays(arg1 >>> 0, arg2, arg3);
    };
    imports.wbg.__wbindgen_number_new = function(arg0) {
        const ret = arg0;
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_new_0394642eae39db16 = function() {
        const ret = new Array();
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_drawBuffers_a606a498b5eaba73 = function(arg0, arg1) {
        getObject(arg0).drawBuffers(getObject(arg1));
    };
    imports.wbg.__wbg_push_109cfc26d02582dd = function(arg0, arg1) {
        const ret = getObject(arg0).push(getObject(arg1));
        return ret;
    };
    imports.wbg.__wbg_of_8276164206196da5 = function(arg0) {
        const ret = Array.of(getObject(arg0));
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_drawBuffersWEBGL_12ce572b8f8b9543 = function(arg0, arg1) {
        getObject(arg0).drawBuffersWEBGL(getObject(arg1));
    };
    imports.wbg.__wbg_enable_775a58f3b9a1b326 = function(arg0, arg1) {
        getObject(arg0).enable(arg1 >>> 0);
    };
    imports.wbg.__wbg_enable_bf2cc1d283856f62 = function(arg0, arg1) {
        getObject(arg0).enable(arg1 >>> 0);
    };
    imports.wbg.__wbg_framebufferTexture2D_1b1b218c5aedac67 = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).framebufferTexture2D(arg1 >>> 0, arg2 >>> 0, arg3 >>> 0, getObject(arg4), arg5);
    };
    imports.wbg.__wbg_framebufferTexture2D_55062a40659a599c = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).framebufferTexture2D(arg1 >>> 0, arg2 >>> 0, arg3 >>> 0, getObject(arg4), arg5);
    };
    imports.wbg.__wbg_framebufferTextureLayer_dd8c4427fdc80b42 = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).framebufferTextureLayer(arg1 >>> 0, arg2 >>> 0, getObject(arg3), arg4, arg5);
    };
    imports.wbg.__wbg_getIndexedParameter_6e0c38c96b72faa0 = function() { return handleError(function (arg0, arg1, arg2) {
        const ret = getObject(arg0).getIndexedParameter(arg1 >>> 0, arg2 >>> 0);
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_getUniformLocation_bf364d4e0ea4bf7d = function(arg0, arg1, arg2, arg3) {
        const ret = getObject(arg0).getUniformLocation(getObject(arg1), getStringFromWasm0(arg2, arg3));
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_getUniformLocation_dc97cf2e79f5657a = function(arg0, arg1, arg2, arg3) {
        const ret = getObject(arg0).getUniformLocation(getObject(arg1), getStringFromWasm0(arg2, arg3));
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_samplerParameterf_594ccbeb4d28a71d = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).samplerParameterf(getObject(arg1), arg2 >>> 0, arg3);
    };
    imports.wbg.__wbg_samplerParameteri_e8bc530ad3f8610a = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).samplerParameteri(getObject(arg1), arg2 >>> 0, arg3);
    };
    imports.wbg.__wbindgen_memory = function() {
        const ret = wasm.memory;
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_buffer_55ba7a6b1b92e2ac = function(arg0) {
        const ret = getObject(arg0).buffer;
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_newwithbyteoffsetandlength_6889cd5c48bfda26 = function(arg0, arg1, arg2) {
        const ret = new Int16Array(getObject(arg0), arg1 >>> 0, arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_newwithbyteoffsetandlength_051cff28f608b6cb = function(arg0, arg1, arg2) {
        const ret = new Uint16Array(getObject(arg0), arg1 >>> 0, arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_newwithbyteoffsetandlength_23602c96f86e952a = function(arg0, arg1, arg2) {
        const ret = new Int32Array(getObject(arg0), arg1 >>> 0, arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_newwithbyteoffsetandlength_fae0a69639559788 = function(arg0, arg1, arg2) {
        const ret = new Uint32Array(getObject(arg0), arg1 >>> 0, arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_newwithbyteoffsetandlength_ab5b524f83702d8d = function(arg0, arg1, arg2) {
        const ret = new Float32Array(getObject(arg0), arg1 >>> 0, arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_texStorage2D_71aadb66c5b0efe9 = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).texStorage2D(arg1 >>> 0, arg2, arg3 >>> 0, arg4, arg5);
    };
    imports.wbg.__wbg_colorMask_a98440726363cd47 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).colorMask(arg1 !== 0, arg2 !== 0, arg3 !== 0, arg4 !== 0);
    };
    imports.wbg.__wbg_colorMask_90441088f82ffdbc = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).colorMask(arg1 !== 0, arg2 !== 0, arg3 !== 0, arg4 !== 0);
    };
    imports.wbg.__wbg_depthMask_78b53ee0d02b2318 = function(arg0, arg1) {
        getObject(arg0).depthMask(arg1 !== 0);
    };
    imports.wbg.__wbg_depthMask_d704601ae2c54009 = function(arg0, arg1) {
        getObject(arg0).depthMask(arg1 !== 0);
    };
    imports.wbg.__wbg_bindTexture_b5b5a8f409c98094 = function(arg0, arg1, arg2) {
        getObject(arg0).bindTexture(arg1 >>> 0, getObject(arg2));
    };
    imports.wbg.__wbg_bindTexture_2170432a0e1e96a5 = function(arg0, arg1, arg2) {
        getObject(arg0).bindTexture(arg1 >>> 0, getObject(arg2));
    };
    imports.wbg.__wbg_bindSampler_5a79668eabf16324 = function(arg0, arg1, arg2) {
        getObject(arg0).bindSampler(arg1 >>> 0, getObject(arg2));
    };
    imports.wbg.__wbg_activeTexture_daa91f21403ceec5 = function(arg0, arg1) {
        getObject(arg0).activeTexture(arg1 >>> 0);
    };
    imports.wbg.__wbg_activeTexture_4c5fbecb8a6751c7 = function(arg0, arg1) {
        getObject(arg0).activeTexture(arg1 >>> 0);
    };
    imports.wbg.__wbg_texParameteri_022fc2aa60490893 = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).texParameteri(arg1 >>> 0, arg2 >>> 0, arg3);
    };
    imports.wbg.__wbg_texParameteri_2b17c6b2eb162904 = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).texParameteri(arg1 >>> 0, arg2 >>> 0, arg3);
    };
    imports.wbg.__wbg_texSubImage2D_f533ccf419af9305 = function() { return handleError(function (arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9) {
        getObject(arg0).texSubImage2D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7 >>> 0, arg8 >>> 0, arg9);
    }, arguments) };
    imports.wbg.__wbg_texSubImage2D_c14bf28d07ec5c5b = function() { return handleError(function (arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9) {
        getObject(arg0).texSubImage2D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7 >>> 0, arg8 >>> 0, getObject(arg9));
    }, arguments) };
    imports.wbg.__wbg_texSubImage2D_22bc2cb44685219e = function() { return handleError(function (arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9) {
        getObject(arg0).texSubImage2D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7 >>> 0, arg8 >>> 0, getObject(arg9));
    }, arguments) };
    imports.wbg.__wbg_compressedTexSubImage2D_bb25b419487cecde = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9) {
        getObject(arg0).compressedTexSubImage2D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7 >>> 0, arg8, arg9);
    };
    imports.wbg.__wbg_compressedTexSubImage2D_cb17efff875f36d3 = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) {
        getObject(arg0).compressedTexSubImage2D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7 >>> 0, getObject(arg8));
    };
    imports.wbg.__wbg_compressedTexSubImage2D_d3d089f5ae7e8252 = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) {
        getObject(arg0).compressedTexSubImage2D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7 >>> 0, getObject(arg8));
    };
    imports.wbg.__wbg_texSubImage3D_78050c0c88c07ffc = function() { return handleError(function (arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11) {
        getObject(arg0).texSubImage3D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9 >>> 0, arg10 >>> 0, arg11);
    }, arguments) };
    imports.wbg.__wbg_texSubImage3D_9483167c9841e4a6 = function() { return handleError(function (arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11) {
        getObject(arg0).texSubImage3D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9 >>> 0, arg10 >>> 0, getObject(arg11));
    }, arguments) };
    imports.wbg.__wbg_compressedTexSubImage3D_2e87e12b0b9694db = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10, arg11) {
        getObject(arg0).compressedTexSubImage3D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9 >>> 0, arg10, arg11);
    };
    imports.wbg.__wbg_compressedTexSubImage3D_7a3dc255a391e422 = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10) {
        getObject(arg0).compressedTexSubImage3D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9 >>> 0, getObject(arg10));
    };
    imports.wbg.__wbg_vertexAttribPointer_b299ee54f90f0edd = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6) {
        getObject(arg0).vertexAttribPointer(arg1 >>> 0, arg2, arg3 >>> 0, arg4 !== 0, arg5, arg6);
    };
    imports.wbg.__wbg_vertexAttribPointer_15414a8557f85fde = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6) {
        getObject(arg0).vertexAttribPointer(arg1 >>> 0, arg2, arg3 >>> 0, arg4 !== 0, arg5, arg6);
    };
    imports.wbg.__wbindgen_string_new = function(arg0, arg1) {
        const ret = getStringFromWasm0(arg0, arg1);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_error_c04eeace266436b4 = function(arg0, arg1) {
        console.error(getObject(arg0), getObject(arg1));
    };
    imports.wbg.__wbg_exitFullscreen_4dc8f9960c8aaf99 = function(arg0) {
        getObject(arg0).exitFullscreen();
    };
    imports.wbg.__wbg_addEventListener_80f44f0373a4ddcf = function() { return handleError(function (arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).addEventListener(getStringFromWasm0(arg1, arg2), getObject(arg3), getObject(arg4));
    }, arguments) };
    imports.wbg.__wbindgen_object_clone_ref = function(arg0) {
        const ret = getObject(arg0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_createVertexArray_de40c88a3bf0369a = function(arg0) {
        const ret = getObject(arg0).createVertexArray();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_createVertexArrayOES_32c8b44c6ff37ac6 = function(arg0) {
        const ret = getObject(arg0).createVertexArrayOES();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_bindVertexArrayOES_b1d7feba24fa1935 = function(arg0, arg1) {
        getObject(arg0).bindVertexArrayOES(getObject(arg1));
    };
    imports.wbg.__wbg_bindVertexArray_48cb510596ac473e = function(arg0, arg1) {
        getObject(arg0).bindVertexArray(getObject(arg1));
    };
    imports.wbg.__wbg_bufferData_8eb7406b0cf28b09 = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).bufferData(arg1 >>> 0, getObject(arg2), arg3 >>> 0);
    };
    imports.wbg.__wbg_bufferData_1a4454da4ff413cc = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).bufferData(arg1 >>> 0, getObject(arg2), arg3 >>> 0);
    };
    imports.wbg.__wbg_querySelector_95f9f997363209a7 = function() { return handleError(function (arg0, arg1, arg2) {
        const ret = getObject(arg0).querySelector(getStringFromWasm0(arg1, arg2));
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_instanceof_HtmlCanvasElement_b94545433bb4d2ef = function(arg0) {
        let result;
        try {
            result = getObject(arg0) instanceof HTMLCanvasElement;
        } catch {
            result = false;
        }
        const ret = result;
        return ret;
    };
    imports.wbg.__wbg_createElement_5ca641a382492ca8 = function() { return handleError(function (arg0, arg1, arg2) {
        const ret = getObject(arg0).createElement(getStringFromWasm0(arg1, arg2));
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_get_36bc7edcd5f7782e = function(arg0, arg1, arg2) {
        const ret = getObject(arg0)[getStringFromWasm0(arg1, arg2)];
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_new_2b6fea4ea03b1b95 = function() {
        const ret = new Object();
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_width_20743a3d4f804928 = function(arg0) {
        const ret = getObject(arg0).width;
        return ret;
    };
    imports.wbg.__wbg_height_5ab1306d8d18baf4 = function(arg0) {
        const ret = getObject(arg0).height;
        return ret;
    };
    imports.wbg.__wbg_body_2ac9821051552b0f = function(arg0) {
        const ret = getObject(arg0).body;
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_appendChild_1f71ff9f4c25cb99 = function() { return handleError(function (arg0, arg1) {
        const ret = getObject(arg0).appendChild(getObject(arg1));
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_requestAnimationFrame_c6f7d52346edca29 = function() { return handleError(function (arg0, arg1) {
        const ret = getObject(arg0).requestAnimationFrame(getObject(arg1));
        return ret;
    }, arguments) };
    imports.wbg.__wbg_stopPropagation_dddca50cbd8775c6 = function(arg0) {
        getObject(arg0).stopPropagation();
    };
    imports.wbg.__wbg_cancelBubble_b05386dd0a9e9bbf = function(arg0) {
        const ret = getObject(arg0).cancelBubble;
        return ret;
    };
    imports.wbg.__wbg_innerWidth_76fb141cbf9cb1e4 = function() { return handleError(function (arg0) {
        const ret = getObject(arg0).innerWidth;
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_innerHeight_5d9e25fc9940bfa5 = function() { return handleError(function (arg0) {
        const ret = getObject(arg0).innerHeight;
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_preventDefault_4e2c657e9c01d7fc = function(arg0) {
        getObject(arg0).preventDefault();
    };
    imports.wbg.__wbg_deltaX_de032cf3a1827b26 = function(arg0) {
        const ret = getObject(arg0).deltaX;
        return ret;
    };
    imports.wbg.__wbg_deltaY_b7960b06b4a2bd76 = function(arg0) {
        const ret = getObject(arg0).deltaY;
        return ret;
    };
    imports.wbg.__wbg_deltaMode_caed80fc4f06f6c4 = function(arg0) {
        const ret = getObject(arg0).deltaMode;
        return ret;
    };
    imports.wbg.__wbg_getModifierState_4476a237c5176bff = function(arg0, arg1, arg2) {
        const ret = getObject(arg0).getModifierState(getStringFromWasm0(arg1, arg2));
        return ret;
    };
    imports.wbg.__wbg_pointerId_ce63681710863137 = function(arg0) {
        const ret = getObject(arg0).pointerId;
        return ret;
    };
    imports.wbg.__wbg_setPointerCapture_4e4f3aafb82450f3 = function() { return handleError(function (arg0, arg1) {
        getObject(arg0).setPointerCapture(arg1);
    }, arguments) };
    imports.wbg.__wbg_buttons_49a61c8517c86f55 = function(arg0) {
        const ret = getObject(arg0).buttons;
        return ret;
    };
    imports.wbg.__wbg_getBoundingClientRect_e547e9868e29e399 = function(arg0) {
        const ret = getObject(arg0).getBoundingClientRect();
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_clientX_e7fc47bc7520b746 = function(arg0) {
        const ret = getObject(arg0).clientX;
        return ret;
    };
    imports.wbg.__wbg_x_79e594166156ad78 = function(arg0) {
        const ret = getObject(arg0).x;
        return ret;
    };
    imports.wbg.__wbg_clientY_cdcfa2d448faa205 = function(arg0) {
        const ret = getObject(arg0).clientY;
        return ret;
    };
    imports.wbg.__wbg_y_2a9fb251aabe4931 = function(arg0) {
        const ret = getObject(arg0).y;
        return ret;
    };
    imports.wbg.__wbg_get_f53c921291c381bd = function() { return handleError(function (arg0, arg1) {
        const ret = Reflect.get(getObject(arg0), getObject(arg1));
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_now_e58d2f1c4b481e88 = function(arg0) {
        const ret = getObject(arg0).now();
        return ret;
    };
    imports.wbg.__wbg_self_742dd6eab3e9211e = function() { return handleError(function () {
        const ret = self.self;
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_window_c409e731db53a0e2 = function() { return handleError(function () {
        const ret = window.window;
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_globalThis_b70c095388441f2d = function() { return handleError(function () {
        const ret = globalThis.globalThis;
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_global_1c72617491ed7194 = function() { return handleError(function () {
        const ret = global.global;
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbindgen_is_undefined = function(arg0) {
        const ret = getObject(arg0) === undefined;
        return ret;
    };
    imports.wbg.__wbg_newnoargs_c9e6043b8ad84109 = function(arg0, arg1) {
        const ret = new Function(getStringFromWasm0(arg0, arg1));
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_call_557a2f2deacc4912 = function() { return handleError(function (arg0, arg1) {
        const ret = getObject(arg0).call(getObject(arg1));
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_is_20a2e5c82eecc47d = function(arg0, arg1) {
        const ret = Object.is(getObject(arg0), getObject(arg1));
        return ret;
    };
    imports.wbg.__wbg_newwithbyteoffsetandlength_abbb764f85f16b6b = function(arg0, arg1, arg2) {
        const ret = new Int8Array(getObject(arg0), arg1 >>> 0, arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_newwithbyteoffsetandlength_88d1d8be5df94b9b = function(arg0, arg1, arg2) {
        const ret = new Uint8Array(getObject(arg0), arg1 >>> 0, arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_set_07da13cc24b69217 = function() { return handleError(function (arg0, arg1, arg2) {
        const ret = Reflect.set(getObject(arg0), getObject(arg1), getObject(arg2));
        return ret;
    }, arguments) };
    imports.wbg.__wbindgen_debug_string = function(arg0, arg1) {
        const ret = debugString(getObject(arg1));
        const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len1 = WASM_VECTOR_LEN;
        getInt32Memory0()[arg0 / 4 + 1] = len1;
        getInt32Memory0()[arg0 / 4 + 0] = ptr1;
    };
    imports.wbg.__wbindgen_throw = function(arg0, arg1) {
        throw new Error(getStringFromWasm0(arg0, arg1));
    };
    imports.wbg.__wbg_resolve_ae38ad63c43ff98b = function(arg0) {
        const ret = Promise.resolve(getObject(arg0));
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_then_8df675b8bb5d5e3c = function(arg0, arg1) {
        const ret = getObject(arg0).then(getObject(arg1));
        return addHeapObject(ret);
    };
    imports.wbg.__wbindgen_cb_drop = function(arg0) {
        const obj = takeObject(arg0).original;
        if (obj.cnt-- == 1) {
            obj.a = 0;
            return true;
        }
        const ret = false;
        return ret;
    };
    imports.wbg.__wbg_performance_96613d016ade8f4f = function(arg0) {
        const ret = getObject(arg0).performance;
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_getExtension_20f7c113f94964b9 = function() { return handleError(function (arg0, arg1, arg2) {
        const ret = getObject(arg0).getExtension(getStringFromWasm0(arg1, arg2));
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_getParameter_d540906c5375b1d1 = function() { return handleError(function (arg0, arg1) {
        const ret = getObject(arg0).getParameter(arg1 >>> 0);
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_getProgramParameter_f96710addeb4a569 = function(arg0, arg1, arg2) {
        const ret = getObject(arg0).getProgramParameter(getObject(arg1), arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_document_d0bac061933b26c1 = function(arg0) {
        const ret = getObject(arg0).document;
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_setTimeout_e69b495baa51d892 = function() { return handleError(function (arg0, arg1, arg2) {
        const ret = getObject(arg0).setTimeout(getObject(arg1), arg2);
        return ret;
    }, arguments) };
    imports.wbg.__wbg_name_06baebeab3141a5e = function(arg0, arg1) {
        const ret = getObject(arg1).name;
        const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len1 = WASM_VECTOR_LEN;
        getInt32Memory0()[arg0 / 4 + 1] = len1;
        getInt32Memory0()[arg0 / 4 + 0] = ptr1;
    };
    imports.wbg.__wbg_instanceof_Window_0e6c0f1096d66c3c = function(arg0) {
        let result;
        try {
            result = getObject(arg0) instanceof Window;
        } catch {
            result = false;
        }
        const ret = result;
        return ret;
    };
    imports.wbg.__wbg_target_25d0a0af0588ba99 = function(arg0) {
        const ret = getObject(arg0).target;
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_altKey_450360bccefdbfb1 = function(arg0) {
        const ret = getObject(arg0).altKey;
        return ret;
    };
    imports.wbg.__wbg_ctrlKey_369252bc3040041f = function(arg0) {
        const ret = getObject(arg0).ctrlKey;
        return ret;
    };
    imports.wbg.__wbg_key_8f799b48d97db561 = function(arg0, arg1) {
        const ret = getObject(arg1).key;
        const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len1 = WASM_VECTOR_LEN;
        getInt32Memory0()[arg0 / 4 + 1] = len1;
        getInt32Memory0()[arg0 / 4 + 0] = ptr1;
    };
    imports.wbg.__wbg_matches_56e9d83fd664e34f = function(arg0) {
        const ret = getObject(arg0).matches;
        return ret;
    };
    imports.wbg.__wbg_requestFullscreen_a502f428375a9f6d = function() { return handleError(function (arg0) {
        getObject(arg0).requestFullscreen();
    }, arguments) };
    imports.wbg.__wbg_setAttribute_16d7681c5ae6bc09 = function() { return handleError(function (arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).setAttribute(getStringFromWasm0(arg1, arg2), getStringFromWasm0(arg3, arg4));
    }, arguments) };
    imports.wbg.__wbg_getParameter_55b5c45524d44b9c = function() { return handleError(function (arg0, arg1) {
        const ret = getObject(arg0).getParameter(arg1 >>> 0);
        return addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_getProgramParameter_c3c357dd0f328a20 = function(arg0, arg1, arg2) {
        const ret = getObject(arg0).getProgramParameter(getObject(arg1), arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_addEventListener_28b7bd2588b5d3f8 = function() { return handleError(function (arg0, arg1, arg2, arg3) {
        getObject(arg0).addEventListener(getStringFromWasm0(arg1, arg2), getObject(arg3));
    }, arguments) };
    imports.wbg.__wbg_removeEventListener_59fa74a031c5fc1d = function() { return handleError(function (arg0, arg1, arg2, arg3) {
        getObject(arg0).removeEventListener(getStringFromWasm0(arg1, arg2), getObject(arg3));
    }, arguments) };
    imports.wbg.__wbg_deleteVertexArray_88ff45a0afd3e542 = function(arg0, arg1) {
        getObject(arg0).deleteVertexArray(getObject(arg1));
    };
    imports.wbg.__wbg_deleteVertexArrayOES_e7632ad3b988cc57 = function(arg0, arg1) {
        getObject(arg0).deleteVertexArrayOES(getObject(arg1));
    };
    imports.wbg.__wbg_clientWaitSync_3f6600969db8421e = function(arg0, arg1, arg2, arg3) {
        const ret = getObject(arg0).clientWaitSync(getObject(arg1), arg2 >>> 0, arg3 >>> 0);
        return ret;
    };
    imports.wbg.__wbg_createRenderbuffer_a8081f3f47a5474e = function(arg0) {
        const ret = getObject(arg0).createRenderbuffer();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_createRenderbuffer_bd0ebb84352e959c = function(arg0) {
        const ret = getObject(arg0).createRenderbuffer();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_renderbufferStorage_b9d8cee541f3741a = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).renderbufferStorage(arg1 >>> 0, arg2 >>> 0, arg3, arg4);
    };
    imports.wbg.__wbg_renderbufferStorage_1d69e2da1af2baf0 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).renderbufferStorage(arg1 >>> 0, arg2 >>> 0, arg3, arg4);
    };
    imports.wbg.__wbg_renderbufferStorageMultisample_d5392684ea6df760 = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).renderbufferStorageMultisample(arg1 >>> 0, arg2, arg3 >>> 0, arg4, arg5);
    };
    imports.wbg.__wbg_texStorage3D_d06f53c309d7cdb2 = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6) {
        getObject(arg0).texStorage3D(arg1 >>> 0, arg2, arg3 >>> 0, arg4, arg5, arg6);
    };
    imports.wbg.__wbg_createSampler_7a6a515fd97fec06 = function(arg0) {
        const ret = getObject(arg0).createSampler();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_getProgramInfoLog_927ea6ba0be3adf8 = function(arg0, arg1, arg2) {
        const ret = getObject(arg1).getProgramInfoLog(getObject(arg2));
        var ptr1 = isLikeNone(ret) ? 0 : passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        var len1 = WASM_VECTOR_LEN;
        getInt32Memory0()[arg0 / 4 + 1] = len1;
        getInt32Memory0()[arg0 / 4 + 0] = ptr1;
    };
    imports.wbg.__wbg_getProgramInfoLog_ca215a5b893b5371 = function(arg0, arg1, arg2) {
        const ret = getObject(arg1).getProgramInfoLog(getObject(arg2));
        var ptr1 = isLikeNone(ret) ? 0 : passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        var len1 = WASM_VECTOR_LEN;
        getInt32Memory0()[arg0 / 4 + 1] = len1;
        getInt32Memory0()[arg0 / 4 + 0] = ptr1;
    };
    imports.wbg.__wbg_getUniformBlockIndex_0c918dab20956ced = function(arg0, arg1, arg2, arg3) {
        const ret = getObject(arg0).getUniformBlockIndex(getObject(arg1), getStringFromWasm0(arg2, arg3));
        return ret;
    };
    imports.wbg.__wbg_uniformBlockBinding_3091fa35b65a2e80 = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).uniformBlockBinding(getObject(arg1), arg2 >>> 0, arg3 >>> 0);
    };
    imports.wbg.__wbg_uniform1i_3211929c5f0ecdf8 = function(arg0, arg1, arg2) {
        getObject(arg0).uniform1i(getObject(arg1), arg2);
    };
    imports.wbg.__wbg_uniform1i_52e7375418604a1d = function(arg0, arg1, arg2) {
        getObject(arg0).uniform1i(getObject(arg1), arg2);
    };
    imports.wbg.__wbg_getActiveUniform_9372a821eb6ce93b = function(arg0, arg1, arg2) {
        const ret = getObject(arg0).getActiveUniform(getObject(arg1), arg2 >>> 0);
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_size_3a09f1de2d605bb9 = function(arg0) {
        const ret = getObject(arg0).size;
        return ret;
    };
    imports.wbg.__wbg_type_6fdd517b967ef329 = function(arg0) {
        const ret = getObject(arg0).type;
        return ret;
    };
    imports.wbg.__wbg_getActiveUniform_8c77b4cae264ca97 = function(arg0, arg1, arg2) {
        const ret = getObject(arg0).getActiveUniform(getObject(arg1), arg2 >>> 0);
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_bufferData_7cac83e3a5ada7eb = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).bufferData(arg1 >>> 0, arg2, arg3 >>> 0);
    };
    imports.wbg.__wbg_bufferData_113c513969656209 = function(arg0, arg1, arg2, arg3) {
        getObject(arg0).bufferData(arg1 >>> 0, arg2, arg3 >>> 0);
    };
    imports.wbg.__wbg_deleteRenderbuffer_6674ca97077e256e = function(arg0, arg1) {
        getObject(arg0).deleteRenderbuffer(getObject(arg1));
    };
    imports.wbg.__wbg_deleteRenderbuffer_a400807e90d141f4 = function(arg0, arg1) {
        getObject(arg0).deleteRenderbuffer(getObject(arg1));
    };
    imports.wbg.__wbg_deleteSampler_25642dd486330545 = function(arg0, arg1) {
        getObject(arg0).deleteSampler(getObject(arg1));
    };
    imports.wbg.__wbg_getShaderParameter_4fa494b1bf9174c4 = function(arg0, arg1, arg2) {
        const ret = getObject(arg0).getShaderParameter(getObject(arg1), arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_getShaderParameter_d89d2e675f0220cc = function(arg0, arg1, arg2) {
        const ret = getObject(arg0).getShaderParameter(getObject(arg1), arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_getShaderInfoLog_013586abb9b0c778 = function(arg0, arg1, arg2) {
        const ret = getObject(arg1).getShaderInfoLog(getObject(arg2));
        var ptr1 = isLikeNone(ret) ? 0 : passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        var len1 = WASM_VECTOR_LEN;
        getInt32Memory0()[arg0 / 4 + 1] = len1;
        getInt32Memory0()[arg0 / 4 + 0] = ptr1;
    };
    imports.wbg.__wbg_getShaderInfoLog_7bb704c665e4a07b = function(arg0, arg1, arg2) {
        const ret = getObject(arg1).getShaderInfoLog(getObject(arg2));
        var ptr1 = isLikeNone(ret) ? 0 : passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        var len1 = WASM_VECTOR_LEN;
        getInt32Memory0()[arg0 / 4 + 1] = len1;
        getInt32Memory0()[arg0 / 4 + 0] = ptr1;
    };
    imports.wbg.__wbg_deleteQuery_5c47fdb30fc406c2 = function(arg0, arg1) {
        getObject(arg0).deleteQuery(getObject(arg1));
    };
    imports.wbg.__wbg_getSyncParameter_f6072b718dac6d0e = function(arg0, arg1, arg2) {
        const ret = getObject(arg0).getSyncParameter(getObject(arg1), arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_fenceSync_76d44c37501be82c = function(arg0, arg1, arg2) {
        const ret = getObject(arg0).fenceSync(arg1 >>> 0, arg2 >>> 0);
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_beginQuery_ef35c4dfd46accdf = function(arg0, arg1, arg2) {
        getObject(arg0).beginQuery(arg1 >>> 0, getObject(arg2));
    };
    imports.wbg.__wbg_endQuery_6c39eb3f5c986272 = function(arg0, arg1) {
        getObject(arg0).endQuery(arg1 >>> 0);
    };
    imports.wbg.__wbg_getQueryParameter_45e87466c1060904 = function(arg0, arg1, arg2) {
        const ret = getObject(arg0).getQueryParameter(getObject(arg1), arg2 >>> 0);
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_readPixels_f9457193dac71ec7 = function() { return handleError(function (arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7) {
        getObject(arg0).readPixels(arg1, arg2, arg3, arg4, arg5 >>> 0, arg6 >>> 0, getObject(arg7));
    }, arguments) };
    imports.wbg.__wbg_readPixels_e0a916a31379045b = function() { return handleError(function (arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7) {
        getObject(arg0).readPixels(arg1, arg2, arg3, arg4, arg5 >>> 0, arg6 >>> 0, getObject(arg7));
    }, arguments) };
    imports.wbg.__wbg_readPixels_08de6bc2c42ac374 = function() { return handleError(function (arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7) {
        getObject(arg0).readPixels(arg1, arg2, arg3, arg4, arg5 >>> 0, arg6 >>> 0, arg7);
    }, arguments) };
    imports.wbg.__wbg_copyTexSubImage2D_2e2dfd80e59ad6a9 = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) {
        getObject(arg0).copyTexSubImage2D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7, arg8);
    };
    imports.wbg.__wbg_copyTexSubImage2D_c81c17de8ed3c47f = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) {
        getObject(arg0).copyTexSubImage2D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7, arg8);
    };
    imports.wbg.__wbg_copyTexSubImage3D_16d4991641ca6614 = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9) {
        getObject(arg0).copyTexSubImage3D(arg1 >>> 0, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9);
    };
    imports.wbg.__wbg_drawElementsInstancedANGLE_a7d44a23728f80ee = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).drawElementsInstancedANGLE(arg1 >>> 0, arg2, arg3 >>> 0, arg4, arg5);
    };
    imports.wbg.__wbg_drawArraysInstancedANGLE_c9bc9f12348d06e3 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).drawArraysInstancedANGLE(arg1 >>> 0, arg2, arg3, arg4);
    };
    imports.wbg.__wbg_drawElementsInstanced_11a1ac661a5f99f9 = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).drawElementsInstanced(arg1 >>> 0, arg2, arg3 >>> 0, arg4, arg5);
    };
    imports.wbg.__wbg_drawArraysInstanced_d554a17d36f42f0f = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).drawArraysInstanced(arg1 >>> 0, arg2, arg3, arg4);
    };
    imports.wbg.__wbg_drawElements_2360851bbd4a1d8a = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).drawElements(arg1 >>> 0, arg2, arg3 >>> 0, arg4);
    };
    imports.wbg.__wbg_drawElements_696f52008122f622 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).drawElements(arg1 >>> 0, arg2, arg3 >>> 0, arg4);
    };
    imports.wbg.__wbg_stencilMask_0ce096eb2d1e1a73 = function(arg0, arg1) {
        getObject(arg0).stencilMask(arg1 >>> 0);
    };
    imports.wbg.__wbg_stencilMask_788bc17f620fa0bd = function(arg0, arg1) {
        getObject(arg0).stencilMask(arg1 >>> 0);
    };
    imports.wbg.__wbg_blendEquation_b210f3bfeaa49a00 = function(arg0, arg1) {
        getObject(arg0).blendEquation(arg1 >>> 0);
    };
    imports.wbg.__wbg_blendEquation_49163b1ebb7544d6 = function(arg0, arg1) {
        getObject(arg0).blendEquation(arg1 >>> 0);
    };
    imports.wbg.__wbg_blendFunc_41a0c6368ac2caff = function(arg0, arg1, arg2) {
        getObject(arg0).blendFunc(arg1 >>> 0, arg2 >>> 0);
    };
    imports.wbg.__wbg_blendFunc_90c0be39b3c65c66 = function(arg0, arg1, arg2) {
        getObject(arg0).blendFunc(arg1 >>> 0, arg2 >>> 0);
    };
    imports.wbg.__wbg_blendEquationSeparate_d204cc8fd6ef5102 = function(arg0, arg1, arg2) {
        getObject(arg0).blendEquationSeparate(arg1 >>> 0, arg2 >>> 0);
    };
    imports.wbg.__wbg_blendEquationSeparate_ff552993763a137f = function(arg0, arg1, arg2) {
        getObject(arg0).blendEquationSeparate(arg1 >>> 0, arg2 >>> 0);
    };
    imports.wbg.__wbg_blendFuncSeparate_d2fd07369172a02a = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).blendFuncSeparate(arg1 >>> 0, arg2 >>> 0, arg3 >>> 0, arg4 >>> 0);
    };
    imports.wbg.__wbg_blendFuncSeparate_868c933d69f5cbee = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).blendFuncSeparate(arg1 >>> 0, arg2 >>> 0, arg3 >>> 0, arg4 >>> 0);
    };
    imports.wbg.__wbg_bindBufferRange_dddda598033d750c = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).bindBufferRange(arg1 >>> 0, arg2 >>> 0, getObject(arg3), arg4, arg5);
    };
    imports.wbg.__wbg_blendColor_0cb2d192801d08a8 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).blendColor(arg1, arg2, arg3, arg4);
    };
    imports.wbg.__wbg_blendColor_ae4688b71eacef4d = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).blendColor(arg1, arg2, arg3, arg4);
    };
    imports.wbg.__wbg_frontFace_533c0549afff2573 = function(arg0, arg1) {
        getObject(arg0).frontFace(arg1 >>> 0);
    };
    imports.wbg.__wbg_frontFace_0905393d4b1ad4a0 = function(arg0, arg1) {
        getObject(arg0).frontFace(arg1 >>> 0);
    };
    imports.wbg.__wbg_cullFace_58c6b913af68087e = function(arg0, arg1) {
        getObject(arg0).cullFace(arg1 >>> 0);
    };
    imports.wbg.__wbg_cullFace_d49665bfe254f335 = function(arg0, arg1) {
        getObject(arg0).cullFace(arg1 >>> 0);
    };
    imports.wbg.__wbg_disableVertexAttribArray_a67a305e4481c274 = function(arg0, arg1) {
        getObject(arg0).disableVertexAttribArray(arg1 >>> 0);
    };
    imports.wbg.__wbg_disableVertexAttribArray_1072f4917261f605 = function(arg0, arg1) {
        getObject(arg0).disableVertexAttribArray(arg1 >>> 0);
    };
    imports.wbg.__wbg_enableVertexAttribArray_fae26009e050884f = function(arg0, arg1) {
        getObject(arg0).enableVertexAttribArray(arg1 >>> 0);
    };
    imports.wbg.__wbg_enableVertexAttribArray_950e67d457cc33d3 = function(arg0, arg1) {
        getObject(arg0).enableVertexAttribArray(arg1 >>> 0);
    };
    imports.wbg.__wbg_vertexAttribIPointer_0636565263d98371 = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).vertexAttribIPointer(arg1 >>> 0, arg2, arg3 >>> 0, arg4, arg5);
    };
    imports.wbg.__wbg_vertexAttribDivisorANGLE_81ba99694cd357ff = function(arg0, arg1, arg2) {
        getObject(arg0).vertexAttribDivisorANGLE(arg1 >>> 0, arg2 >>> 0);
    };
    imports.wbg.__wbg_vertexAttribDivisor_5398f4fb31a5d688 = function(arg0, arg1, arg2) {
        getObject(arg0).vertexAttribDivisor(arg1 >>> 0, arg2 >>> 0);
    };
    imports.wbg.__wbg_polygonOffset_9ae22cd23f6484ce = function(arg0, arg1, arg2) {
        getObject(arg0).polygonOffset(arg1, arg2);
    };
    imports.wbg.__wbg_polygonOffset_4be51deceaeb05a6 = function(arg0, arg1, arg2) {
        getObject(arg0).polygonOffset(arg1, arg2);
    };
    imports.wbg.__wbg_depthFunc_d51318dc36b94359 = function(arg0, arg1) {
        getObject(arg0).depthFunc(arg1 >>> 0);
    };
    imports.wbg.__wbg_depthFunc_b3a31ea87ce7184a = function(arg0, arg1) {
        getObject(arg0).depthFunc(arg1 >>> 0);
    };
    imports.wbg.__wbg_stencilMaskSeparate_a054d1d320fe0206 = function(arg0, arg1, arg2) {
        getObject(arg0).stencilMaskSeparate(arg1 >>> 0, arg2 >>> 0);
    };
    imports.wbg.__wbg_stencilMaskSeparate_2c14306ff80dd128 = function(arg0, arg1, arg2) {
        getObject(arg0).stencilMaskSeparate(arg1 >>> 0, arg2 >>> 0);
    };
    imports.wbg.__wbg_stencilOpSeparate_45ce637b6c214467 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).stencilOpSeparate(arg1 >>> 0, arg2 >>> 0, arg3 >>> 0, arg4 >>> 0);
    };
    imports.wbg.__wbg_stencilOpSeparate_89c1a7ecf95be129 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).stencilOpSeparate(arg1 >>> 0, arg2 >>> 0, arg3 >>> 0, arg4 >>> 0);
    };
    imports.wbg.__wbg_stencilFuncSeparate_239feed28c1d1713 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).stencilFuncSeparate(arg1 >>> 0, arg2 >>> 0, arg3, arg4 >>> 0);
    };
    imports.wbg.__wbg_stencilFuncSeparate_974e35f5fd349f9e = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).stencilFuncSeparate(arg1 >>> 0, arg2 >>> 0, arg3, arg4 >>> 0);
    };
    imports.wbg.__wbg_scissor_6300d9c04a530564 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).scissor(arg1, arg2, arg3, arg4);
    };
    imports.wbg.__wbg_scissor_63dfa92ea1ca440b = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).scissor(arg1, arg2, arg3, arg4);
    };
    imports.wbg.__wbg_viewport_dec03e2220fd7c60 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).viewport(arg1, arg2, arg3, arg4);
    };
    imports.wbg.__wbg_viewport_2ea457ddc071f65c = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).viewport(arg1, arg2, arg3, arg4);
    };
    imports.wbg.__wbg_depthRange_2592e8900efffdd2 = function(arg0, arg1, arg2) {
        getObject(arg0).depthRange(arg1, arg2);
    };
    imports.wbg.__wbg_depthRange_adcf9f07d2d00772 = function(arg0, arg1, arg2) {
        getObject(arg0).depthRange(arg1, arg2);
    };
    imports.wbg.__wbg_clearBufferuiv_8f7179aece72942c = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).clearBufferuiv(arg1 >>> 0, arg2, getArrayU32FromWasm0(arg3, arg4));
    };
    imports.wbg.__wbg_uniform4f_40e6c4d73e2a4d4e = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).uniform4f(getObject(arg1), arg2, arg3, arg4, arg5);
    };
    imports.wbg.__wbg_uniform4f_af0baa78d4637b1c = function(arg0, arg1, arg2, arg3, arg4, arg5) {
        getObject(arg0).uniform4f(getObject(arg1), arg2, arg3, arg4, arg5);
    };
    imports.wbg.__wbg_invalidateFramebuffer_f9560d9f8de9d073 = function() { return handleError(function (arg0, arg1, arg2) {
        getObject(arg0).invalidateFramebuffer(arg1 >>> 0, getObject(arg2));
    }, arguments) };
    imports.wbg.__wbg_readBuffer_13c8ea4f7c603ce4 = function(arg0, arg1) {
        getObject(arg0).readBuffer(arg1 >>> 0);
    };
    imports.wbg.__wbg_blitFramebuffer_80af15adba6372b4 = function(arg0, arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9, arg10) {
        getObject(arg0).blitFramebuffer(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8, arg9 >>> 0, arg10 >>> 0);
    };
    imports.wbg.__wbg_framebufferRenderbuffer_115cdbae1fe83488 = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).framebufferRenderbuffer(arg1 >>> 0, arg2 >>> 0, arg3 >>> 0, getObject(arg4));
    };
    imports.wbg.__wbg_framebufferRenderbuffer_43aab255682840eb = function(arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).framebufferRenderbuffer(arg1 >>> 0, arg2 >>> 0, arg3 >>> 0, getObject(arg4));
    };
    imports.wbg.__wbg_getContext_99e77f282516dee7 = function() { return handleError(function (arg0, arg1, arg2, arg3) {
        const ret = getObject(arg0).getContext(getStringFromWasm0(arg1, arg2), getObject(arg3));
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_instanceof_WebGl2RenderingContext_e29e70ae6c00bfdd = function(arg0) {
        let result;
        try {
            result = getObject(arg0) instanceof WebGL2RenderingContext;
        } catch {
            result = false;
        }
        const ret = result;
        return ret;
    };
    imports.wbg.__wbg_getSupportedExtensions_b0a904f246e1e119 = function(arg0) {
        const ret = getObject(arg0).getSupportedExtensions();
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_length_820c786973abdd8a = function(arg0) {
        const ret = getObject(arg0).length;
        return ret;
    };
    imports.wbg.__wbg_get_7303ed2ef026b2f5 = function(arg0, arg1) {
        const ret = getObject(arg0)[arg1 >>> 0];
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_button_aa31df90c614ac2f = function(arg0) {
        const ret = getObject(arg0).button;
        return ret;
    };
    imports.wbg.__wbg_shiftKey_63109f31fffaae88 = function(arg0) {
        const ret = getObject(arg0).shiftKey;
        return ret;
    };
    imports.wbg.__wbg_ctrlKey_41177ea5ca2d0e20 = function(arg0) {
        const ret = getObject(arg0).ctrlKey;
        return ret;
    };
    imports.wbg.__wbg_altKey_c88db7062e567dc3 = function(arg0) {
        const ret = getObject(arg0).altKey;
        return ret;
    };
    imports.wbg.__wbg_metaKey_5e5a70bdf96145ac = function(arg0) {
        const ret = getObject(arg0).metaKey;
        return ret;
    };
    imports.wbg.__wbg_offsetX_9cd27b438e5897e1 = function(arg0) {
        const ret = getObject(arg0).offsetX;
        return ret;
    };
    imports.wbg.__wbg_offsetY_2b6db1a63d48871b = function(arg0) {
        const ret = getObject(arg0).offsetY;
        return ret;
    };
    imports.wbg.__wbg_movementX_3582aabddf312baa = function(arg0) {
        const ret = getObject(arg0).movementX;
        return ret;
    };
    imports.wbg.__wbg_movementY_003849633a9c93ba = function(arg0) {
        const ret = getObject(arg0).movementY;
        return ret;
    };
    imports.wbg.__wbg_keyCode_2f432c9e03114433 = function(arg0) {
        const ret = getObject(arg0).keyCode;
        return ret;
    };
    imports.wbg.__wbg_charCode_47114a6dc513450a = function(arg0) {
        const ret = getObject(arg0).charCode;
        return ret;
    };
    imports.wbg.__wbg_code_b35334b0fb28bae5 = function(arg0, arg1) {
        const ret = getObject(arg1).code;
        const ptr1 = passStringToWasm0(ret, wasm.__wbindgen_malloc, wasm.__wbindgen_realloc);
        const len1 = WASM_VECTOR_LEN;
        getInt32Memory0()[arg0 / 4 + 1] = len1;
        getInt32Memory0()[arg0 / 4 + 0] = ptr1;
    };
    imports.wbg.__wbg_shiftKey_938c211e3ca8bef3 = function(arg0) {
        const ret = getObject(arg0).shiftKey;
        return ret;
    };
    imports.wbg.__wbg_metaKey_41b042131f94a501 = function(arg0) {
        const ret = getObject(arg0).metaKey;
        return ret;
    };
    imports.wbg.__wbg_devicePixelRatio_433a57ae570165c6 = function(arg0) {
        const ret = getObject(arg0).devicePixelRatio;
        return ret;
    };
    imports.wbg.__wbg_setwidth_18ec5eda4c4617b4 = function(arg0, arg1) {
        getObject(arg0).width = arg1 >>> 0;
    };
    imports.wbg.__wbg_setheight_56ca229310ba885a = function(arg0, arg1) {
        getObject(arg0).height = arg1 >>> 0;
    };
    imports.wbg.__wbg_style_502d8f9e29c68b63 = function(arg0) {
        const ret = getObject(arg0).style;
        return addHeapObject(ret);
    };
    imports.wbg.__wbg_setProperty_28d579f9c15848c5 = function() { return handleError(function (arg0, arg1, arg2, arg3, arg4) {
        getObject(arg0).setProperty(getStringFromWasm0(arg1, arg2), getStringFromWasm0(arg3, arg4));
    }, arguments) };
    imports.wbg.__wbg_fullscreenElement_20a494bdb171be54 = function(arg0) {
        const ret = getObject(arg0).fullscreenElement;
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    };
    imports.wbg.__wbg_matches_b0a85bab6120e8dc = function(arg0) {
        const ret = getObject(arg0).matches;
        return ret;
    };
    imports.wbg.__wbg_matchMedia_f1f54434b496d226 = function() { return handleError(function (arg0, arg1, arg2) {
        const ret = getObject(arg0).matchMedia(getStringFromWasm0(arg1, arg2));
        return isLikeNone(ret) ? 0 : addHeapObject(ret);
    }, arguments) };
    imports.wbg.__wbg_addListener_fc6dcad032f66c29 = function() { return handleError(function (arg0, arg1) {
        getObject(arg0).addListener(getObject(arg1));
    }, arguments) };
    imports.wbg.__wbg_removeListener_67c8d28b6b20f37f = function() { return handleError(function (arg0, arg1) {
        getObject(arg0).removeListener(getObject(arg1));
    }, arguments) };
    imports.wbg.__wbg_clearTimeout_4d1e10d1de1c1ac9 = function(arg0, arg1) {
        getObject(arg0).clearTimeout(arg1);
    };
    imports.wbg.__wbg_cancelAnimationFrame_97993243fbac902e = function() { return handleError(function (arg0, arg1) {
        getObject(arg0).cancelAnimationFrame(arg1);
    }, arguments) };
    imports.wbg.__wbindgen_closure_wrapper2170 = function(arg0, arg1, arg2) {
        const ret = makeMutClosure(arg0, arg1, 90, __wbg_adapter_26);
        return addHeapObject(ret);
    };
    imports.wbg.__wbindgen_closure_wrapper2176 = function(arg0, arg1, arg2) {
        const ret = makeMutClosure(arg0, arg1, 90, __wbg_adapter_26);
        return addHeapObject(ret);
    };
    imports.wbg.__wbindgen_closure_wrapper2179 = function(arg0, arg1, arg2) {
        const ret = makeMutClosure(arg0, arg1, 90, __wbg_adapter_26);
        return addHeapObject(ret);
    };
    imports.wbg.__wbindgen_closure_wrapper2182 = function(arg0, arg1, arg2) {
        const ret = makeMutClosure(arg0, arg1, 90, __wbg_adapter_26);
        return addHeapObject(ret);
    };
    imports.wbg.__wbindgen_closure_wrapper2185 = function(arg0, arg1, arg2) {
        const ret = makeMutClosure(arg0, arg1, 95, __wbg_adapter_26);
        return addHeapObject(ret);
    };
    imports.wbg.__wbindgen_closure_wrapper2189 = function(arg0, arg1, arg2) {
        const ret = makeMutClosure(arg0, arg1, 90, __wbg_adapter_26);
        return addHeapObject(ret);
    };
    imports.wbg.__wbindgen_closure_wrapper2192 = function(arg0, arg1, arg2) {
        const ret = makeMutClosure(arg0, arg1, 90, __wbg_adapter_26);
        return addHeapObject(ret);
    };
    imports.wbg.__wbindgen_closure_wrapper2195 = function(arg0, arg1, arg2) {
        const ret = makeMutClosure(arg0, arg1, 90, __wbg_adapter_26);
        return addHeapObject(ret);
    };
    imports.wbg.__wbindgen_closure_wrapper2391 = function(arg0, arg1, arg2) {
        const ret = makeMutClosure(arg0, arg1, 95, __wbg_adapter_43);
        return addHeapObject(ret);
    };
    imports.wbg.__wbindgen_closure_wrapper5216 = function(arg0, arg1, arg2) {
        const ret = makeMutClosure(arg0, arg1, 95, __wbg_adapter_26);
        return addHeapObject(ret);
    };

    return imports;
}

function __wbg_init_memory(imports, maybe_memory) {

}

function __wbg_finalize_init(instance, module) {
    wasm = instance.exports;
    __wbg_init.__wbindgen_wasm_module = module;
    cachedFloat32Memory0 = null;
    cachedFloat64Memory0 = null;
    cachedInt32Memory0 = null;
    cachedUint32Memory0 = null;
    cachedUint8Memory0 = null;

    wasm.__wbindgen_start();
    return wasm;
}

function initSync(module) {
    if (wasm !== undefined) return wasm;

    const imports = __wbg_get_imports();

    __wbg_init_memory(imports);

    if (!(module instanceof WebAssembly.Module)) {
        module = new WebAssembly.Module(module);
    }

    const instance = new WebAssembly.Instance(module, imports);

    return __wbg_finalize_init(instance, module);
}

async function __wbg_init(input) {
    if (wasm !== undefined) return wasm;

    if (typeof input === 'undefined') {
        input = new URL('grid_bg.wasm', import.meta.url);
    }
    const imports = __wbg_get_imports();

    if (typeof input === 'string' || (typeof Request === 'function' && input instanceof Request) || (typeof URL === 'function' && input instanceof URL)) {
        input = fetch(input);
    }

    __wbg_init_memory(imports);

    const { instance, module } = await __wbg_load(await input, imports);

    return __wbg_finalize_init(instance, module);
}

export { initSync }
export default __wbg_init;
