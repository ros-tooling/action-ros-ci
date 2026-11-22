import eslint from "@eslint/js";
import { defineConfig, globalIgnores } from "eslint/config";
import eslintPluginPrettierRecommended from "eslint-plugin-prettier/recommended";
import globals from "globals";
import tseslint from "typescript-eslint";

export default defineConfig(
	globalIgnores(["dist/", "coverage/"]),
	eslint.configs.recommended,
	tseslint.configs.base,
	tseslint.configs.eslintRecommended,
	{
		languageOptions: {
			ecmaVersion: 2022,
			sourceType: "module",
			globals: {
				...globals.node,
			},
		},
	},
	eslintPluginPrettierRecommended,
);
