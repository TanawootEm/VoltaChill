import java.util.Properties

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

// Signing key details live outside the repo (~/.android/scanlab-signing.properties).
val signProps = Properties().apply {
    val f = file(System.getProperty("user.home") + "/.android/scanlab-signing.properties")
    if (f.exists()) f.inputStream().use { load(it) }
}

android {
    namespace = "com.tanawoot.scanlab"
    compileSdk = 35
    ndkVersion = "27.2.12479018"

    defaultConfig {
        applicationId = "com.tanawoot.scanlab"
        minSdk = 24
        targetSdk = 34
        versionCode = 2
        versionName = "0.2.0"
        ndk { abiFilters += listOf("arm64-v8a") }
    }
    buildFeatures { buildConfig = true }
    // Two APKs from one codebase: phone (portrait) and car (DiLink head unit, landscape:
    // smaller UI + room for the climate bar). Same package id: each device gets one of them.
    flavorDimensions += "target"
    productFlavors {
        create("phone") {
            dimension = "target"
            buildConfigField("boolean", "CAR", "false")
        }
        create("car") {
            dimension = "target"
            versionNameSuffix = "-car"
            buildConfigField("boolean", "CAR", "true")
        }
    }
    signingConfigs {
        if (!signProps.isEmpty) {
            create("voltachill") {
                storeFile = file(signProps.getProperty("storeFile"))
                storePassword = signProps.getProperty("storePassword")
                keyAlias = signProps.getProperty("keyAlias")
                keyPassword = signProps.getProperty("keyPassword")
            }
        }
    }
    buildTypes {
        // debug and release share one key, so either installs over the other
        val key = signingConfigs.findByName("voltachill")
        getByName("release") {
            isMinifyEnabled = false
            if (key != null) signingConfig = key
        }
        getByName("debug") {
            isMinifyEnabled = false
            if (key != null) signingConfig = key
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions { jvmTarget = "17" }
    // .so is built by ../build-apk.sh (cargo-ndk) into src/main/jniLibs
}

dependencies {
    implementation("androidx.games:games-activity:3.0.5")
    // GameActivity extends AppCompatActivity and implements OnApplyWindowInsetsListener
    implementation("androidx.appcompat:appcompat:1.7.0")
    implementation("androidx.core:core-ktx:1.13.1")
}
