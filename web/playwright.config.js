import { defineConfig } from "@playwright/test";

export default defineConfig({
  testDir: "./tests", fullyParallel: false, workers: 1,
  use: { baseURL: "http://127.0.0.1:8123", headless: true,
    launchOptions: { ...(process.env.CHROMIUM_PATH ? { executablePath: process.env.CHROMIUM_PATH } : {}),
      args: ["--use-angle=swiftshader", "--enable-unsafe-swiftshader"] } },
  webServer: { command: "node tests/server.mjs", url: "http://127.0.0.1:8123", reuseExistingServer: false },
});
